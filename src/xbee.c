/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2015-2020 David Sainty
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* $Id$ */

/*
 * avrdude interface for AVR devices Over-The-Air programmable via an
 * XBee Series 2 device.
 *
 * The XBee programmer is STK500v1 (optiboot) encapsulated in the XBee
 * API protocol.  The bootloader supporting this protocol is available at:
 *
 * https://github.com/davidsainty/xbeeboot
 */

#include "ac_cfg.h"

#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "stk500_private.h"
#include "stk500.h"
#include "xbee.h"

/*
 * After eight seconds the AVR bootloader watchdog will kick in.  But
 * to allow for the possibility of eight seconds upstream and another
 * eight seconds downstream, allow for 16 retries (of roughly one
 * second each).
 */
#ifndef XBEE_MAX_RETRIES
#define XBEE_MAX_RETRIES 16
#endif

/*
 * Maximum chunk size, which is the maximum encapsulated payload to be
 * delivered to the remote CPU.
 *
 * There is an additional overhead of 3 bytes encapsulation, one
 * "REQUEST" byte, one sequence number byte, and one
 * "FIRMWARE_DELIVER" request type.
 *
 * The ZigBee maximum (unfragmented) payload is 84 bytes.  Source
 * routing decreases that by two bytes overhead, plus two bytes per
 * hop.  Maximum hop support is for 11 or 25 hops depending on
 * firmware.
 *
 * Network layer encryption decreases the maximum payload by 18 bytes.
 * APS end-to-end encryption decreases the maximum payload by 9 bytes.
 * Both these layers are available in concert, as seen in the section
 * "Network and APS layer encryption", decreasing our maximum payload
 * by both 18 bytes and 9 bytes.
 *
 * Our maximum payload size should therefore ideally be 84 - 18 - 9 =
 * 57 bytes, and therefore a chunk size of 54 bytes for zero hops.
 *
 * Source: XBee X2C manual: "Maximum RF payload size" section for most
 * details; "Network layer encryption and decryption" section for the
 * reference to 18 bytes of overhead; and "Enable APS encryption" for
 * the reference to 9 bytes of overhead.
 */
#ifndef XBEEBOOT_MAX_CHUNK
#define XBEEBOOT_MAX_CHUNK 54
#endif

/*
 * Maximum source route intermediate hops.  This is described in the
 * documentation variously as 40 hops (routing table); OR 25 hops
 * (firmware 4x58 or later); OR 11 hops (firmware earlier than 4x58).
 *
 * What isn't described is how to know if a given source route length
 * is actually supported by the mesh for our target device.
 */
#ifndef XBEE_MAX_INTERMEDIATE_HOPS
#define XBEE_MAX_INTERMEDIATE_HOPS 40
#endif

/* Protocol */
#define XBEEBOOT_PACKET_TYPE_ACK 0
#define XBEEBOOT_PACKET_TYPE_REQUEST 1

/*
 * Read signature bytes - Direct copy of the Arduino behaviour to
 * satisfy Optiboot.
 */
static int xbee_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {
  unsigned char buf[32];

  /* Signature byte reads are always 3 bytes. */

  if (m->size < 3) {
    pmsg_error("memsize too small for sig byte read\n");
    return -1;
  }

  buf[0] = Cmnd_STK_READ_SIGN;
  buf[1] = Sync_CRC_EOP;

  serial_send(&pgm->fd, buf, 2);

  if (serial_recv(&pgm->fd, buf, 5) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    pmsg_error("programmer is out of sync\n");
    return -1;
  } else if (buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }
  if (buf[4] != Resp_STK_OK) {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[4]);
    return -3;
  }

  m->buf[0] = buf[1];
  m->buf[1] = buf[2];
  m->buf[2] = buf[3];

  return 3;
}

struct XBeeSequenceStatistics {
  struct timeval sendTime;
};

struct XBeeStaticticsSummary {
  struct timeval minimum;
  struct timeval maximum;
  struct timeval sum;
  unsigned long samples;
};

#define XBEE_STATS_GROUPS 4
#define XBEE_STATS_FRAME_LOCAL 0
#define XBEE_STATS_FRAME_REMOTE 1
#define XBEE_STATS_TRANSMIT 2
#define XBEE_STATS_RECEIVE 3

static const char* groupNames[] =
  {
   "FRAME_LOCAL",
   "FRAME_REMOTE",
   "TRANSMIT",
   "RECEIVE"
  };

struct XBeeBootSession {
  struct serial_device *serialDevice;
  union filedescriptor serialDescriptor;

  unsigned char xbee_address[10];
  int directMode;
  unsigned char outSequence;
  unsigned char inSequence;

  /*
   * XBee API frame sequence number.
   */
  unsigned char txSequence;

  /*
   * Set to non-zero if the transport is broken to the point it is
   * considered unusable.
   */
  int transportUnusable;

  int xbeeResetPin;

  size_t inInIndex;
  size_t inOutIndex;
  unsigned char inBuffer[256];

  int sourceRouteHops; /* -1 if unset */
  int sourceRouteChanged;

  /*
   * The source route is an array of intermediate 16 bit addresses,
   * starting with the address nearest to the target address, and
   * finishing with the address closest to our local device.
   */
  unsigned char sourceRoute[2 * XBEE_MAX_INTERMEDIATE_HOPS];

  struct XBeeSequenceStatistics sequenceStatistics[256 * XBEE_STATS_GROUPS];
  struct XBeeStaticticsSummary groupSummary[XBEE_STATS_GROUPS];
};

static void xbeeStatsReset(struct XBeeStaticticsSummary *summary)
{
  summary->minimum.tv_sec = 0;
  summary->minimum.tv_usec = 0;
  summary->maximum.tv_sec = 0;
  summary->maximum.tv_usec = 0;
  summary->sum.tv_sec = 0;
  summary->sum.tv_usec = 0;
  summary->samples = 0;
}

static void xbeeStatsAdd(struct XBeeStaticticsSummary *summary,
                         struct timeval const *sample)
{
  summary->sum.tv_usec += sample->tv_usec;
  if (summary->sum.tv_usec > 1000000) {
    summary->sum.tv_usec -= 1000000;
    summary->sum.tv_sec++;
  }
  summary->sum.tv_sec += sample->tv_sec;

  if (summary->samples == 0 ||
      summary->minimum.tv_sec > sample->tv_sec ||
      (summary->minimum.tv_sec == sample->tv_sec &&
       summary->minimum.tv_usec > sample->tv_usec)) {
    summary->minimum = *sample;
  }

  if (summary->maximum.tv_sec < sample->tv_sec ||
      (summary->maximum.tv_sec == sample->tv_sec &&
       summary->maximum.tv_usec < sample->tv_usec)) {
    summary->maximum = *sample;
  }

  summary->samples++;
}

static void xbeeStatsSummarise(struct XBeeStaticticsSummary const *summary)
{
  pmsg_notice("  Minimum response time: %lu.%06lu\n", summary->minimum.tv_sec, summary->minimum.tv_usec);
  pmsg_notice("  Maximum response time: %lu.%06lu\n", summary->maximum.tv_sec, summary->maximum.tv_usec);

  struct timeval average;

  const unsigned long samples = summary->samples;
  average.tv_sec = summary->sum.tv_sec / samples;

  unsigned long long usecs = summary->sum.tv_usec;
  usecs += (summary->sum.tv_sec % samples) * 1000000;
  usecs = usecs / samples;
  average.tv_sec += usecs / 1000000;
  average.tv_usec = usecs % 1000000;

  pmsg_notice("  Average response time: %lu.%06lu\n", average.tv_sec, average.tv_usec);
}

static void XBeeBootSessionInit(struct XBeeBootSession *xbs) {
  xbs->serialDevice = &serial_serdev;
  xbs->directMode = 1;
  xbs->xbeeResetPin = XBEE_DEFAULT_RESET_PIN;
  xbs->outSequence = 0;
  xbs->inSequence = 0;
  xbs->txSequence = 0;
  xbs->transportUnusable = 0;
  xbs->inInIndex = 0;
  xbs->inOutIndex = 0;
  xbs->sourceRouteHops = -1;
  xbs->sourceRouteChanged = 0;

  int group;
  for (group = 0; group < 3; group++) {
    int index;
    for (index = 0; index < 256; index++)
      xbs->sequenceStatistics[group * 256 + index].sendTime.tv_sec = (time_t)0;
    xbeeStatsReset(&xbs->groupSummary[group]);
  }
}

#define xbeebootsession(fdp) (struct XBeeBootSession*)((fdp)->pfd)

static void xbeedev_setresetpin(const union filedescriptor *fdp, int xbeeResetPin)
{
  struct XBeeBootSession *xbs = xbeebootsession(fdp);
  xbs->xbeeResetPin = xbeeResetPin;
}

enum xbee_stat_is_retry_enum {XBEE_STATS_NOT_RETRY, XBEE_STATS_IS_RETRY};
typedef enum xbee_stat_is_retry_enum xbee_stat_is_retry;

static void xbeedev_stats_send(struct XBeeBootSession *xbs,
                               char const *detail,
                               int detailSequence,
                               unsigned int group, unsigned char sequence,
                               xbee_stat_is_retry retry,
                               struct timeval const *sendTime)
{
  struct XBeeSequenceStatistics *stats =
    &xbs->sequenceStatistics[group * 256 + sequence];

  if (retry == XBEE_STATS_NOT_RETRY)
    stats->sendTime = *sendTime;

  if (detailSequence >= 0) {
    pmsg_notice2("stats: send Group %s Sequence %u : "
      "Send %lu.%06lu %s Sequence %d\n",
      groupNames[group],
      (unsigned int) sequence,
      (unsigned long) sendTime->tv_sec,
      (unsigned long) sendTime->tv_usec,
      detail, detailSequence);
  } else {
    pmsg_notice2("stats: send Group %s Sequence %u : "
      "Send %lu.%06lu %s\n",
      groupNames[group],
      (unsigned int) sequence,
      (unsigned long) sendTime->tv_sec,
      (unsigned long) sendTime->tv_usec,
      detail);
  }
}

static void xbeedev_stats_receive(struct XBeeBootSession *xbs,
                                  char const *detail,
                                  unsigned int group, unsigned char sequence,
                                  struct timeval const *receiveTime)
{
  struct XBeeSequenceStatistics *stats =
    &xbs->sequenceStatistics[group * 256 + sequence];
  struct timeval delay;
  time_t secs;
  long usecs;

  secs = receiveTime->tv_sec - stats->sendTime.tv_sec;
  usecs = receiveTime->tv_usec - stats->sendTime.tv_usec;

  if (usecs < 0) {
    usecs += 1000000;
    secs--;
  }

  delay.tv_sec = secs;
  delay.tv_usec = usecs;

  pmsg_notice2("stats: receive Group %s Sequence %u : "
    "Send %lu.%06lu Receive %lu.%06lu Delay %lu.%06lu %s\n",
    groupNames[group],
    (unsigned int) sequence,
    (unsigned long) stats->sendTime.tv_sec,
    (unsigned long) stats->sendTime.tv_usec,
    (unsigned long) receiveTime->tv_sec,
    (unsigned long) receiveTime->tv_usec,
    (unsigned long) secs,
    (unsigned long) usecs,
    detail);

  xbeeStatsAdd(&xbs->groupSummary[group], &delay);
}

static int sendAPIRequest(struct XBeeBootSession *xbs,
                          unsigned char apiType,
                          int txSequence,
                          int apiOption,
                          int prePayload1,
                          int prePayload2,
                          int packetType,
                          int sequence,
                          int appType,
                          char const *detail,
                          int detailSequence,
                          unsigned int frameGroup,
                          xbee_stat_is_retry retry,
                          unsigned int dataLength,
                          const unsigned char *data)
{
  unsigned char frame[256];

  unsigned char *fp = &frame[5];
  unsigned char *dataStart = fp;
  unsigned char checksum = 0xff;
  unsigned char length = 0;
  struct timeval time;

  gettimeofday(&time, NULL);

  pmsg_notice2("sendAPIRequest(): %lu.%06lu %d, %d, %d, %d %s\n",
    (unsigned long) time.tv_sec,
    (unsigned long) time.tv_usec,
    (int) packetType, (int)sequence, appType,
    data == NULL ? -1 : (int)*data, detail);

#define fpput(x)                                                \
  do {                                                          \
    const unsigned char v = (x);                                \
    if (v == 0x7d || v == 0x7e || v == 0x11 || v == 0x13) {     \
      *fp++ = 0x7d;                                             \
      *fp++ = v ^ 0x20;                                         \
    } else {                                                    \
      *fp++ = v;                                                \
    }                                                           \
    checksum -= v;                                              \
    length++;                                                   \
  } while (0)

  fpput(apiType); /* ZigBee Receive Packet or ZigBee Transmit Request */

  if (apiOption >= 0)
    fpput(apiOption); /* Receive options (RX) */

  if (txSequence >= 0) {
    fpput(txSequence); /* Delivery sequence (TX/AT) */

    /*
     * Record the frame send time.  Note that frame sequences are
     * never retries.
     */
    xbeedev_stats_send(xbs, detail, detailSequence,
                       frameGroup, txSequence, 0, &time);
  }

  if (apiType != 0x08) {
    /* Automatically inhibit addressing for local AT command requests. */
    size_t index;
    for (index = 0; index < 10; index++) {
      const unsigned char val = xbs->xbee_address[index];
      fpput(val);
    }

    /*
     * If this is an API call with remote address, but is not a Create
     * Source Route request, consider prefixing it with source routing
     * instructions.
     */
    if (apiType != 0x21 && xbs->sourceRouteChanged) {
      pmsg_notice2("sendAPIRequest(): issuing Create Source Route request with %d hops\n", xbs->sourceRouteHops);

      int rc = sendAPIRequest(xbs, 0x21, /* Create Source Route */
                              0, -1, 0, xbs->sourceRouteHops,
                              -1, -1, -1,
                              "Create Source Route for FRAME_REMOTE",
                              txSequence,
                              XBEE_STATS_FRAME_LOCAL, /* Local, no response */
                              0, /* Not a retry */
                              xbs->sourceRouteHops * 2,
                              xbs->sourceRoute);
      if (rc != 0)
        return rc;

      xbs->sourceRouteChanged = 0;
    }
  }

  if (prePayload1 >= 0)
    fpput(prePayload1); /* Transmit broadcast radius */

  if (prePayload2 >= 0)
    fpput(prePayload2); /* Transmit options */

  if (packetType >= 0)
    fpput(packetType); /* XBEEBOOT_PACKET_TYPE_{ACK,REQUEST} */

  if (sequence >= 0) {
    fpput(sequence);

    /* Record the send time */
    if (packetType == XBEEBOOT_PACKET_TYPE_REQUEST)
      xbeedev_stats_send(xbs, detail, sequence, XBEE_STATS_TRANSMIT,
                         sequence, retry, &time);
  }

  if (appType >= 0)
    fpput(appType); /* FIRMWARE_DELIVER */

  {
    size_t index;
    for (index = 0; index < dataLength; index++)
      fpput(data[index]);
  }

  /* Length BEFORE checksum byte */
  const unsigned char unescapedLength = length;

  fpput(checksum);

  /* Length AFTER checksum byte */
  const unsigned int finalLength = fp - dataStart;

  frame[0] = 0x7e;
  fp = &frame[1];
  fpput(0);
  fpput(unescapedLength);
  const unsigned int prefixLength = fp - frame;
  unsigned char *frameStart = dataStart - prefixLength;
  memmove(frameStart, frame, prefixLength);

  return xbs->serialDevice->send(&xbs->serialDescriptor,
                                 frameStart, finalLength + prefixLength);
}

static int sendPacket(struct XBeeBootSession *xbs,
                      const char *detail,
                      unsigned char packetType,
                      unsigned char sequence,
                      xbee_stat_is_retry retry,
                      int appType,
                      unsigned int dataLength,
                      const unsigned char *data)
{
  unsigned char apiType;
  int prePayload1;
  int prePayload2;

  if (xbs->directMode) {
    /*
     * In direct mode we are pretending to be an XBee device
     * forwarding on data received from the transmitting XBee.  We
     * therefore format the data as a remote XBee would, encapsulated
     * in a 0x90 packet.
     */
    apiType = 0x90; /* ZigBee Receive Packet */
    prePayload1 = -1;
    prePayload2 = -1;
  } else {
    /*
     * In normal mode we are requesting a payload delivery,
     * encapsulated in a 0x10 packet.
     */
    apiType = 0x10; /* ZigBee Transmit Request */
    prePayload1 = 0;
    prePayload2 = 0;
  }

  while ((++xbs->txSequence & 0xff) == 0);
  return sendAPIRequest(xbs, apiType, xbs->txSequence, -1,
                        prePayload1, prePayload2, packetType,
                        sequence, appType,
                        detail, sequence,
                        XBEE_STATS_FRAME_REMOTE, retry,
                        dataLength, data);
}

#define XBEE_LENGTH_LEN 2
#define XBEE_CHECKSUM_LEN 1
#define XBEE_APITYPE_LEN 1
#define XBEE_APISEQUENCE_LEN 1
#define XBEE_ADDRESS_64BIT_LEN 8
#define XBEE_ADDRESS_16BIT_LEN 2
#define XBEE_RADIUS_LEN 1
#define XBEE_TXOPTIONS_LEN 1
#define XBEE_RXOPTIONS_LEN 1

static void xbeedev_record16Bit(struct XBeeBootSession *xbs,
                                const unsigned char *rx16Bit)
{
  /*
   * We don't start out knowing what the 16-bit device address is, but
   * we should receive it on the return packets, and re-use it from
   * that point on.
   */
  unsigned char * const tx16Bit =
    &xbs->xbee_address[XBEE_ADDRESS_64BIT_LEN];
  if (memcmp(rx16Bit, tx16Bit, XBEE_ADDRESS_16BIT_LEN) != 0) {
    pmsg_notice2("xbeedev_record16Bit(): new 16-bit address: %02x%02x\n",
                    (unsigned int)rx16Bit[0],
                    (unsigned int)rx16Bit[1]);
    memcpy(tx16Bit, rx16Bit, XBEE_ADDRESS_16BIT_LEN);
  }
}

/*
 * Return 0 on success.
 * Return -1 on generic error (normally serial timeout).
 * Return -512 + XBee AT Response code
 */
#define XBEE_AT_RETURN_CODE(x) (((x) >= -512 && (x) <= -256) ? (x) + 512 : -1)
static int xbeedev_poll(struct XBeeBootSession *xbs,
                        unsigned char **buf, size_t *buflen,
                        int waitForAck,
                        int waitForSequence)
{
  for (;;) {
    unsigned char byte;
    unsigned char frame[256];
    unsigned int frameSize;

  before_frame:
    do {
      const int rc = xbs->serialDevice->recv(&xbs->serialDescriptor, &byte, 1);
      if (rc < 0)
        return rc;
    } while (byte != 0x7e);

  start_of_frame:
    {
      size_t index = 0;
      int escaped = 0;
      frameSize = XBEE_LENGTH_LEN;
      do {
        const int rc = xbs->serialDevice->recv(&xbs->serialDescriptor,
                                               &byte, 1);
        if (rc < 0)
          return rc;

        if (byte == 0x7e)
          /*
           * No matter when we receive a frame start byte, we should
           * abort parsing and start a fresh frame.
           */
          goto start_of_frame;

        if (escaped) {
          byte ^= 0x20;
          escaped = 0;
        } else if (byte == 0x7d) {
          escaped = 1;
          continue;
        }

        if (index >= sizeof(frame))
          goto before_frame;

        frame[index++] = byte;

        if (index == XBEE_LENGTH_LEN) {
          /* Length plus the two length bytes, plus the checksum byte */
          frameSize = (frame[0] << 8 | frame[1]) +
            XBEE_LENGTH_LEN + XBEE_CHECKSUM_LEN;

          if (frameSize >= sizeof(frame))
            /* Too long - immediately give up on this frame */
            goto before_frame;
        }
      } while (index < frameSize);

      /* End of frame */
      unsigned char checksum = 1;
      size_t cIndex;
      for (cIndex = 2; cIndex < index; cIndex++) {
        checksum += frame[cIndex];
      }

      if (checksum) {
        /* Checksum didn't match */
        pmsg_notice2("xbeedev_poll(): bad checksum %d\n", (int) checksum);
        continue;
      }
    }

    const unsigned char frameType = frame[2];

    struct timeval receiveTime;
    gettimeofday(&receiveTime, NULL);

    pmsg_notice2("xbeedev_poll(): %lu.%06lu Received frame type %x\n",
      (unsigned long) receiveTime.tv_sec,
      (unsigned long) receiveTime.tv_usec,
      (unsigned int) frameType);

    if (frameType == 0x97 && frameSize > 16) {
      /* Remote command response */
      unsigned char txSequence = frame[3];
      unsigned char resultCode = frame[16];

      xbeedev_stats_receive(xbs, "Remote AT command response",
                            XBEE_STATS_FRAME_REMOTE, txSequence, &receiveTime);

      pmsg_notice("xbeedev_poll(): remote command %d result code %d\n",
        (int) txSequence, (int) resultCode);

      if (waitForSequence >= 0 && waitForSequence == frame[3])
        /* Received result for our sequence numbered request */
        return -512 + resultCode;
    } else if (frameType == 0x88 && frameSize > 6) {
      /* Local command response */
      unsigned char txSequence = frame[3];

      xbeedev_stats_receive(xbs, "Local AT command response",
                            XBEE_STATS_FRAME_LOCAL, txSequence, &receiveTime);

      pmsg_notice("xbeedev_poll(): local command %c%c result code %d\n",
        frame[4], frame[5], (int)frame[6]);

      if (waitForSequence >= 0 && waitForSequence == txSequence)
        /* Received result for our sequence numbered request */
        return 0;
    } else if (frameType == 0x8b && frameSize > 7) {
      /* Transmit status */
      unsigned char txSequence = frame[3];

      xbeedev_stats_receive(xbs, "Transmit status", XBEE_STATS_FRAME_REMOTE,
                            txSequence, &receiveTime);

      pmsg_notice2("xbeedev_poll(): transmit status %d result code %d\n",
        (int) frame[3], (int) frame[7]);
    } else if (frameType == 0xa1 &&
               frameSize >= XBEE_LENGTH_LEN + XBEE_APITYPE_LEN +
               XBEE_ADDRESS_64BIT_LEN +
               XBEE_ADDRESS_16BIT_LEN + 2 + XBEE_CHECKSUM_LEN) {
      /* Route Record Indicator */
      if (memcmp(&frame[XBEE_LENGTH_LEN + XBEE_APITYPE_LEN],
                 xbs->xbee_address, XBEE_ADDRESS_64BIT_LEN) != 0) {
        /* Not from our target device */
        pmsg_notice2("xbeedev_poll(): route Record Indicator from other XBee\n");
        continue;
      }

      /*
       * We don't start out knowing what the 16-bit device address is,
       * but we should receive it on the return packets, and re-use it
       * from that point on.
       */
      {
        const unsigned char *rx16Bit =
          &frame[XBEE_LENGTH_LEN + XBEE_APITYPE_LEN +
                 XBEE_ADDRESS_64BIT_LEN];
        xbeedev_record16Bit(xbs, rx16Bit);
      }

      const unsigned int header = XBEE_LENGTH_LEN + XBEE_APITYPE_LEN +
        XBEE_ADDRESS_64BIT_LEN +
        XBEE_ADDRESS_16BIT_LEN;

      const unsigned char receiveOptions = frame[header];
      const unsigned char hops = frame[header + 1];

      pmsg_notice2("xbeedev_poll(): "
        "Route Record Indicator from target XBee: "
        "hops=%d options=%d\n", (int)hops, (int)receiveOptions);

      if (frameSize < header + 2 + hops * 2 + XBEE_CHECKSUM_LEN)
        /* Bounds check: Frame is too small */
        continue;

      const unsigned int tableOffset = header + 2;

      unsigned char index;
      for (index = 0; index < hops; index++) {
        pmsg_notice2("xbeedev_poll(): "
          "Route Intermediate Hop %d : %02x%02x\n", (int)index,
          (int)frame[tableOffset + index * 2],
          (int)frame[tableOffset + index * 2 + 1]);
      }

      if (hops <= XBEE_MAX_INTERMEDIATE_HOPS) {
        if (xbs->sourceRouteHops != (int)hops ||
            memcmp(&frame[tableOffset], xbs->sourceRoute, hops * 2) != 0) {
          memcpy(xbs->sourceRoute, &frame[tableOffset], hops * 2);
          xbs->sourceRouteHops = hops;
          xbs->sourceRouteChanged = 1;

          pmsg_notice2("xbeedev_poll(): route has changed\n");
        }
      }
    } else if (frameType == 0x10 || frameType == 0x90) {
      unsigned char *dataStart;
      unsigned int dataLength;

      if (frameType == 0x10) {
        /* Direct mode frame */
        const unsigned int header = XBEE_LENGTH_LEN +
          XBEE_APITYPE_LEN + XBEE_APISEQUENCE_LEN +
          XBEE_ADDRESS_64BIT_LEN + XBEE_ADDRESS_16BIT_LEN +
          XBEE_RADIUS_LEN + XBEE_TXOPTIONS_LEN;

        if (frameSize <= header + XBEE_CHECKSUM_LEN)
          /* Bounds check: Frame is too small */
          continue;

        dataLength = frameSize - header - XBEE_CHECKSUM_LEN;
        dataStart = &frame[header];
      } else {
        /* Remote reply frame */
        const unsigned int header = XBEE_LENGTH_LEN +
          XBEE_APITYPE_LEN + XBEE_ADDRESS_64BIT_LEN + XBEE_ADDRESS_16BIT_LEN +
          XBEE_RXOPTIONS_LEN;

        if (frameSize <= header + XBEE_CHECKSUM_LEN)
          /* Bounds check: Frame is too small */
          continue;

        dataLength = frameSize - header - XBEE_CHECKSUM_LEN;
        dataStart = &frame[header];

        if (memcmp(&frame[XBEE_LENGTH_LEN + XBEE_APITYPE_LEN],
                   xbs->xbee_address, XBEE_ADDRESS_64BIT_LEN) != 0) {
          /*
           * This packet is not from our target device.  Unlikely
           * to ever happen, but if it does we have to ignore
           * it.
           */
          continue;
        }

        /*
         * We don't start out knowing what the 16-bit device address
         * is, but we should receive it on the return packets, and
         * re-use it from that point on.
         */
        {
          const unsigned char *rx16Bit =
            &frame[XBEE_LENGTH_LEN + XBEE_APITYPE_LEN +
                   XBEE_ADDRESS_64BIT_LEN];
          xbeedev_record16Bit(xbs, rx16Bit);
        }
      }

      if (dataLength >= 2) {
        const unsigned char protocolType = dataStart[0];
        const unsigned char sequence = dataStart[1];

        pmsg_notice2("xbeedev_poll(): "
          "%lu.%06lu Packet %d #%d\n", (unsigned long)receiveTime.tv_sec,
          (unsigned long)receiveTime.tv_usec,
          (int)protocolType, (int)sequence);

        if (protocolType == XBEEBOOT_PACKET_TYPE_ACK) {
          /* ACK */
          xbeedev_stats_receive(xbs, "XBeeBoot ACK",
                                XBEE_STATS_TRANSMIT, sequence,
                                &receiveTime);

          /*
           * We can't update outSequence here, we already do that
           * somewhere else.
           */
          if (waitForAck >= 0 && waitForAck == sequence)
            return 0;
        } else if (protocolType == XBEEBOOT_PACKET_TYPE_REQUEST &&
                   dataLength >= 4 && dataStart[2] == 24) {
          /* REQUEST FRAME_REPLY */
          xbeedev_stats_receive(xbs, "XBeeBoot Receive", XBEE_STATS_RECEIVE,
                                sequence, &receiveTime);

          unsigned char nextSequence = xbs->inSequence;
          while ((++nextSequence & 0xff) == 0);
          if (sequence == nextSequence) {
            xbs->inSequence = nextSequence;

            const size_t textLength = dataLength - 3;
            size_t index;
            for (index = 0; index < textLength; index++) {
              const unsigned char data = dataStart[3 + index];
              if (buflen != NULL && *buflen > 0) {
                /* If we are receiving right now, and have a buffer ... */
                *(*buf)++ = data;
                (*buflen)--;
              } else {
                xbs->inBuffer[xbs->inInIndex++] = data;
                if (xbs->inInIndex == sizeof(xbs->inBuffer))
                  xbs->inInIndex = 0;
                if (xbs->inInIndex == xbs->inOutIndex) {
                  /* Should be impossible */
                  pmsg_error("buffer overrun\n");
                  xbs->transportUnusable = 1;
                  return -1;
                }
              }
            }

            /*msg_error("ACK %x\n", (unsigned int)sequence);*/
            sendPacket(xbs, "Transmit Request ACK for RECEIVE",
                       XBEEBOOT_PACKET_TYPE_ACK, sequence,
                       XBEE_STATS_NOT_RETRY,
                       -1, 0, NULL);

            if (buf != NULL && *buflen == 0)
              /* Input buffer has been filled */
              return 0;

            /*
             * Input buffer has NOT been filled, we are still in a
             * receive.  Not a retry, this is the first point we know
             * for sure for this sequence number.
             */
            while ((++nextSequence & 0xff) == 0);
            xbeedev_stats_send(xbs, "poll() implies pending RECEIVE",
                               nextSequence,
                               XBEE_STATS_RECEIVE,
                               nextSequence, XBEE_STATS_NOT_RETRY,
                               &receiveTime);
          }
        }
      }
    }
  }
}

/*
 * @return
 *          0 on success, a negative value on failure, or a positive
 *          value indicating the sequence number associated with the
 *          request.
 */
static int localAsyncAT(struct XBeeBootSession *xbs, char const *detail,
                        unsigned char at1, unsigned char at2, int value)
{
  if (xbs->directMode)
    /*
     * Remote XBee AT commands make no sense in direct mode - there is
     * no XBee device to communicate with.
     *
     * Return success, no sequence number.
     */
    return 0;

  while ((++xbs->txSequence & 0xff) == 0);
  const unsigned char sequence = xbs->txSequence;

  unsigned char buf[3];
  size_t length = 0;

  buf[length++] = at1;
  buf[length++] = at2;

  if (value >= 0)
    buf[length++] = (unsigned char)value;

  pmsg_notice("local AT command: %c%c\n", at1, at2);

  /* Local AT command 0x08 */
  int rc = sendAPIRequest(xbs, 0x08, sequence, -1, -1, -1, -1, -1, -1,
                          detail, -1, XBEE_STATS_FRAME_LOCAL,
                          XBEE_STATS_NOT_RETRY,
                          length, buf);
  if (rc < 0)
    /* Failed */
    return rc;

  /* Success, positive sequence number */
  return (int)sequence;
}

static int localAT(struct XBeeBootSession *xbs, char const *detail,
                   unsigned char at1, unsigned char at2, int value)
{
  int result = localAsyncAT(xbs, detail, at1, at2, value);

  if (result <= 0)
    /* Failure, or success without a sequence number */
    return result;

  unsigned char sequence = (unsigned char)result;

  int retries;
  for (retries = 0; retries < 5; retries++) {
    const int rc = xbeedev_poll(xbs, NULL, NULL, -1, sequence);
    if (rc == 0)
      return 0;
  }

  return -1;
}

/*
 * Return 0 on success.
 * Return -1 on generic error (normally serial timeout).
 * Return -512 + XBee AT Response code
 */
static int sendAT(struct XBeeBootSession *xbs, char const *detail,
                  unsigned char at1, unsigned char at2, int value)
{
  if (xbs->directMode)
    /*
     * Remote XBee AT commands make no sense in direct mode - there is
     * no XBee device to communicate with.
     */
    return 0;

  while ((++xbs->txSequence & 0xff) == 0);
  const unsigned char sequence = xbs->txSequence;

  unsigned char buf[3];
  size_t length = 0;

  buf[length++] = at1;
  buf[length++] = at2;

  if (value >= 0)
    buf[length++] = (unsigned char)value;

  pmsg_notice("remote AT command: %c%c\n", at1, at2);

  /* Remote AT command 0x17 with Apply Changes 0x02 */
  sendAPIRequest(xbs, 0x17, sequence, -1,
                 -1, -1, -1,
                 0x02, -1,
                 detail, -1, XBEE_STATS_FRAME_REMOTE,
                 XBEE_STATS_NOT_RETRY,
                 length, buf);

  int retries;
  for (retries = 0; retries < 30; retries++) {
    const int rc = xbeedev_poll(xbs, NULL, NULL, -1, sequence);
    const int xbeeRc = XBEE_AT_RETURN_CODE(rc);
    if (xbeeRc == 0)
      /* Translate to normal success code */
      return 0;
    if (rc != -1)
      return rc;
  }

  return -1;
}

/*
 * Return 0 on no error recognised, 1 if error was detected and reported
 */
static int xbeeATError(int rc) {
  const int xbeeRc = XBEE_AT_RETURN_CODE(rc);
  if (xbeeRc < 0)
    return 0;

  if (xbeeRc == 1) {
    pmsg_error("unable to communicate with remote XBee\n");
  } else if (xbeeRc == 2) {
    pmsg_error("remote XBee: invalid command\n");
  } else if (xbeeRc == 3) {
    pmsg_error("remote XBee: invalid command parameter\n");
  } else if (xbeeRc == 4) {
    pmsg_error("remote XBee: transmission failure\n");
  } else {
    pmsg_error("unrecognised remote XBee error code %d\n", xbeeRc);
  }
  return 1;
}

static void xbeedev_free(struct XBeeBootSession *xbs)
{
  xbs->serialDevice->close(&xbs->serialDescriptor);
  free(xbs);
}

static void xbeedev_close(union filedescriptor *fdp)
{
  struct XBeeBootSession *xbs = xbeebootsession(fdp);
  xbeedev_free(xbs);
}

static int xbeedev_open(const char *port, union pinfo pinfo,
                        union filedescriptor *fdp)
{
  /*
   * The syntax for XBee devices is defined as:
   *
   * -P <XBeeAddress>@[serialdevice]
   *
   * ... or ...
   *
   * -P @[serialdevice]
   *
   * ... for a direct connection.
   */
  char *ttySeparator = strchr(port, '@');
  if (ttySeparator == NULL) {
    pmsg_error("XBee: bad port syntax, require <xbee-address>@<serial-device>\n");
    return -1;
  }

  struct XBeeBootSession *xbs = malloc(sizeof(struct XBeeBootSession));
  if (xbs == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }

  XBeeBootSessionInit(xbs);

  char *tty = &ttySeparator[1];

  if (ttySeparator == port) {
    /* Direct connection */
    memset(xbs->xbee_address, 0, 8);
    xbs->directMode = 1;
  } else {
    size_t addrIndex = 0;
    int nybble = -1;
    char const *address = port;
    while (address != ttySeparator) {
      char hex = *address++;
      unsigned int val;
      if (hex >= '0' && hex <= '9') {
        val = hex - '0';
      } else if (hex >= 'A' && hex <= 'F') {
        val = hex - 'A' + 10;
      } else if  (hex >= 'a' && hex <= 'f') {
        val = hex - 'a' + 10;
      } else {
        break;
      }
      if (nybble == -1) {
        nybble = val;
      } else {
        xbs->xbee_address[addrIndex++] = (nybble * 16) | val;
        nybble = -1;
        if (addrIndex == 8)
          break;
      }
    }

    if (addrIndex != 8 || address != ttySeparator || nybble != -1) {
      pmsg_error("XBee: bad XBee address, require 16-character hexadecimal address\n");
      free(xbs);
      return -1;
    }

    xbs->directMode = 0;
  }

  /* Unknown 16 bit address */
  xbs->xbee_address[8] = 0xff;
  xbs->xbee_address[9] = 0xfe;

  pmsg_trace("XBee address: %02x%02x%02x%02x%02x%02x%02x%02x\n",
    (unsigned int) xbs->xbee_address[0],
    (unsigned int) xbs->xbee_address[1],
    (unsigned int) xbs->xbee_address[2],
    (unsigned int) xbs->xbee_address[3],
    (unsigned int) xbs->xbee_address[4],
    (unsigned int) xbs->xbee_address[5],
    (unsigned int) xbs->xbee_address[6],
    (unsigned int) xbs->xbee_address[7]);

  if (pinfo.serialinfo.baud) {
    /*
     * User supplied the correct baud rate.
     */
  } else if (xbs->directMode) {
    /*
     * In direct mode, default to 19200.
     *
     * Why?
     *
     * In this mode, we are NOT talking to an XBee, we are talking
     * directly to an AVR device that thinks it is talking to an XBee
     * itself.
     *
     * Because, an XBee is a 3.3V device defaulting to 9600baud, and
     * the Atmel328P is only rated at a maximum clock rate of 8MHz
     * with a 3.3V supply, so there's a high likelihood a remote
     * Atmel328P will be clocked at 8MHz.
     *
     * With a direct connection, there's a good chance we're talking
     * to an Arduino clocked at 16MHz with an XBee-enabled chip
     * plugged in.  The doubled clock rate means a doubled serial
     * rate.  Double 9600 baud == 19200 baud.
     */
    pinfo.serialinfo.baud = 19200;
  } else {
    /*
     * In normal mode, default to 9600.
     *
     * Why?
     *
     * XBee devices default to 9600 baud.  In this mode we are talking
     * to the XBee device, not the far-end device, so it's the local
     * XBee baud rate we should select.  The baud rate of the AVR
     * device is irrelevant.
     */
    pinfo.serialinfo.baud = 9600;
  }
  pinfo.serialinfo.cflags = SERIAL_8N1;

  pmsg_notice("baud %ld\n", (long)pinfo.serialinfo.baud);

  {
    const int rc = xbs->serialDevice->open(tty, pinfo,
                                           &xbs->serialDescriptor);
    if (rc < 0) {
      free(xbs);
      return rc;
    }
  }

  if (!xbs->directMode) {
    /* Attempt to ensure the local XBee is in API mode 2 */
    {
      const int rc = localAT(xbs, "AT AP=2", 'A', 'P', 2);
      if (rc < 0) {
        pmsg_error("local XBee is not responding\n");
        xbeedev_free(xbs);
        return rc;
      }
    }

    /*
     * At this point we want to set the remote XBee parameters as
     * required for talking to XBeeBoot.  Ideally we would start with
     * an "FR" full reset, but because that causes the XBee to
     * disappear off the mesh for a significant period and become
     * unresponsive, we don't do that.
     */

    /*
     * Issue an "Aggregate Routing Notification" to enable many-to-one
     * routing to this device.  This has two effects:
     *
     * - Establishes a route from the remote XBee attached to the CPU
     *   being programmed back to the local XBee.
     *
     * - Enables the 0xa1 Route frames so that we can make use of
     *   Source Routing to deliver packets directly to the remote
     *   XBee.
     *
     * Under "RF packet routing" subsection "Many-to-One routing", the
     * XBee S2C manual states "Applications that require multiple data
     * collectors can also use many-to-one routing. If more than one
     * data collector device sends a many-to-one broadcast, devices
     * create one reverse routing table entry for each collector."
     *
     * Under "RF packet routing" subsection "Source routing", the XBee
     * S2C manual states "To use source routing, a device must use the
     * API mode, and it must send periodic many-to-one route request
     * broadcasts (AR command) to create a many-to-one route to it on
     * all devices".
     */
    {
      const int rc = localAT(xbs, "AT AR=0", 'A', 'R', 0);
      if (rc < 0) {
        pmsg_error("local XBee is not responding\n");
        xbeedev_free(xbs);
        return rc;
      }
    }

    /*
     * Disable RTS input on the remote XBee, just in case it is
     * enabled by default.  XBeeBoot doesn't attempt to support flow
     * control, and so it may not correctly drive this pin if RTS mode
     * is the default configuration.
     *
     * XBee IO port 6 is the only pin that supports RTS mode, so there
     * is no need to support any alternative pin.
     */
    const int rc = sendAT(xbs, "AT D6=0", 'D', '6', 0);
    if (rc < 0) {
      xbeedev_free(xbs);

      if (xbeeATError(rc))
        return -1;

      pmsg_error("remote XBee is not responding\n");
      return rc;
    }
  }

  fdp->pfd = xbs;

  return 0;
}

static int xbeedev_send(const union filedescriptor *fdp,
                        const unsigned char *buf, size_t buflen)
{
  struct XBeeBootSession *xbs = xbeebootsession(fdp);

  if (xbs->transportUnusable)
    /* Don't attempt to continue on an unusable transport layer */
    return -1;

  while (buflen > 0) {
    unsigned char sequence = xbs->outSequence;
    while ((++sequence & 0xff) == 0);
    xbs->outSequence = sequence;

    /*
     * We are about to send some data, and that might lead potentially
     * to received data before we see the ACK for this transmission.
     * As this might be the trigger seen before the next "recv"
     * operation, record that we have delivered this potential
     * trigger.
     */
    {
      unsigned char nextSequence = xbs->inSequence;
      while ((++nextSequence & 0xff) == 0);

      struct timeval sendTime;
      gettimeofday(&sendTime, NULL);

      /*
       * Optimistic records should never be treated as retries,
       * because they might simply be guessing too optimistically.
       */
      xbeedev_stats_send(xbs, "send() hints possible triggered RECEIVE",
                         nextSequence,
                         XBEE_STATS_RECEIVE,
                         nextSequence, 0, &sendTime);
    }

    /*
     * Chunk the data into chunks of up to XBEEBOOT_MAX_CHUNK bytes.
     */
    unsigned char maximum_chunk = XBEEBOOT_MAX_CHUNK;

    /*
     * Source routing incurs a two byte fixed overhead, plus a two
     * byte additional cost per intermediate hop.
     *
     * We are attempting to avoid fragmentation here, so resize our
     * maximum size to anticipate the overhead of the current number
     * of hops.  If our maximum chunk would be less than one, just
     * give up and hope fragmentation will somehow save us.
     */
    const int hops = xbs->sourceRouteHops;
    if (hops > 0 && (hops * 2 + 2) < XBEEBOOT_MAX_CHUNK)
      maximum_chunk -= hops * 2 + 2;

    const unsigned char blockLength =
      (buflen > maximum_chunk) ? maximum_chunk : buflen;

    int pollRc = 0;

    /* Repeatedly send whilst timing out waiting for ACK responses. */
    int retries;
    for (retries = 0; retries < XBEE_MAX_RETRIES; retries++) {
      int sendRc =
        sendPacket(xbs,
                   "Transmit Request Data, expect ACK for TRANSMIT",
                   XBEEBOOT_PACKET_TYPE_REQUEST, sequence,
                   retries > 0 ? XBEE_STATS_IS_RETRY : XBEE_STATS_NOT_RETRY,
                   23 /* FIRMWARE_DELIVER */,
                   blockLength, buf);
      if (sendRc < 0) {
        /* There is no way to recover from a failure mid-send */
        xbs->transportUnusable = 1;
        return sendRc;
      }

      pollRc = xbeedev_poll(xbs, NULL, NULL, sequence, -1);
      if (pollRc == 0) {
        /* Send was ACK'd */
        buflen -= blockLength;
        buf += blockLength;
        break;
      }

      /*
       * Test the connection to the local XBee by repeatedly
       * requesting local configuration details.  This functionally
       * has no effect, but will allow us to measure any reliability
       * issues on this link.
       */
      localAsyncAT(xbs, "Local XBee ping [send]", 'A', 'P', -1);

      /*
       * If we don't receive an ACK it might be because the chip
       * missed an ACK from us.  Resend that too after a timeout,
       * unless it's zero which is an illegal sequence number.
       */
      if (xbs->inSequence != 0) {
        int ackRc = sendPacket(xbs,
                               "Transmit Request ACK [Retry in send] "
                               "for RECEIVE",
                               XBEEBOOT_PACKET_TYPE_ACK,
                               xbs->inSequence,
                               XBEE_STATS_IS_RETRY,
                               -1, 0, NULL);
        if (ackRc < 0) {
          /* There is no way to recover from a failure mid-send */
          xbs->transportUnusable = 1;
          return ackRc;
        }
      }
    }

    if (pollRc < 0) {
      /* There is no way to recover from a failure mid-send */
      xbs->transportUnusable = 1;
      return pollRc;
    }
  }

  return 0;
}

static int xbeedev_recv(const union filedescriptor *fdp,
                        unsigned char *buf, size_t buflen)
{
  struct XBeeBootSession *xbs = xbeebootsession(fdp);

  /*
   * First de-buffer anything previously received in a chunk that
   * couldn't be immediately delievered.
   */
  while (xbs->inInIndex != xbs->inOutIndex) {
    *buf++ = xbs->inBuffer[xbs->inOutIndex++];
    if (xbs->inOutIndex == sizeof(xbs->inBuffer))
      xbs->inOutIndex = 0;
    if (--buflen == 0)
      return 0;
  }

  if (xbs->transportUnusable)
    /* Don't attempt to continue on an unusable transport layer */
    return -1;

  /*
   * When we expect to receive data, that is the time to start the
   * clock.
   */
  {
    unsigned char nextSequence = xbs->inSequence;
    while ((++nextSequence & 0xff) == 0);

    struct timeval sendTime;
    gettimeofday(&sendTime, NULL);

    /*
     * Not a retry - in fact this is the first stage we know for sure
     * a RECEIVE is due.
     */
    xbeedev_stats_send(xbs, "recv() implies pending RECEIVE",
                       nextSequence,
                       XBEE_STATS_RECEIVE,
                       nextSequence,
                       XBEE_STATS_NOT_RETRY,
                       &sendTime);
  }

  int retries;
  for (retries = 0; retries < XBEE_MAX_RETRIES; retries++) {
    const int rc = xbeedev_poll(xbs, &buf, &buflen, -1, -1);
    if (rc == 0)
      return 0;

    if (xbs->transportUnusable)
      /* Don't attempt to continue on an unusable transport layer */
      return -1;

    /*
     * Test the connection to the local XBee by repeatedly
     * requesting local configuration details.  This functionally
     * has no effect, but will allow us to measure any reliability
     * issues on this link.
     */
    localAsyncAT(xbs, "Local XBee ping [recv]", 'A', 'P', -1);

    /*
     * The chip may have missed an ACK from us.  Resend after a
     * timeout.
     */
    if (xbs->inSequence != 0)
      sendPacket(xbs, "Transmit Request ACK [Retry in recv] for RECEIVE",
                 XBEEBOOT_PACKET_TYPE_ACK, xbs->inSequence,
                 XBEE_STATS_IS_RETRY,
                 -1, 0, NULL);
  }
  return -1;
}

static int xbeedev_drain(const union filedescriptor *fdp, int display)
{
  struct XBeeBootSession *xbs = xbeebootsession(fdp);

  if (xbs->transportUnusable)
    /* Don't attempt to continue on an unusable transport layer */
    return -1;

  /*
   * Flushing the local serial buffer is unhelpful under this protocol
   */
  do {
    xbs->inOutIndex = xbs->inInIndex = 0;
  } while (xbeedev_poll(xbs, NULL, NULL, -1, -1) == 0);

  return 0;
}

static int xbeedev_set_dtr_rts(const union filedescriptor *fdp, int is_on)
{
  struct XBeeBootSession *xbs = xbeebootsession(fdp);

  if (xbs->directMode)
    /* Correct for direct mode */
    return xbs->serialDevice->set_dtr_rts(&xbs->serialDescriptor, is_on);

  /*
   * For non-direct mode (Over-The-Air) we need to issue XBee commands
   * to the remote XBee in order to reset the AVR CPU and initiate the
   * XBeeBoot bootloader.
   */
  const int rc = sendAT(xbs, is_on ? "AT [DTR]=low" : "AT [DTR]=high",
                        'D', '0' + xbs->xbeeResetPin, is_on ? 5 : 4);
  if (rc < 0) {
    if (xbeeATError(rc))
      return -1;

    pmsg_error("remote XBee is not responding\n");
    return rc;
  }

  return 0;
}

/*
 * Device descriptor for XBee framing.
 */
static struct serial_device xbee_serdev_frame = {
  .open = xbeedev_open,
  .close = xbeedev_close,
  .rawclose = xbeedev_close,
  .send = xbeedev_send,
  .recv = xbeedev_recv,
  .drain = xbeedev_drain,
  .set_dtr_rts = xbeedev_set_dtr_rts,
  .flags = SERDEV_FL_NONE,
};

static int xbee_getsync(const PROGRAMMER *pgm) {
  unsigned char buf[2], resp[2];

  /*
   * Issue sync request as per STK500.  Unlike stk500_getsync(), don't
   * retry here - the underlying protocol will deal with retries for
   * us in xbeedev_send() and should be reliable.
   */
  buf[0] = Cmnd_STK_GET_SYNC;
  buf[1] = Sync_CRC_EOP;

  int sendRc = serial_send(&pgm->fd, buf, 2);
  if (sendRc < 0) {
    pmsg_error("unable to deliver STK_GET_SYNC to the remote XBeeBoot bootloader\n");
    return sendRc;
  }

  /*
   * The same is true of the receive - it will retry on timeouts until
   * the response buffer is full.
   */
  int recvRc = serial_recv(&pgm->fd, resp, 2);
  if (recvRc < 0) {
    pmsg_error("no response to STK_GET_SYNC from the remote XBeeBoot bootloader\n");
    return recvRc;
  }

  if (resp[0] != Resp_STK_INSYNC) {
    pmsg_error("not in sync, resp=0x%02x\n", (unsigned int) resp[0]);
    return -1;
  }

  if (resp[1] != Resp_STK_OK) {
    pmsg_error("in sync, not OK, resp=0x%02x\n", (unsigned int) resp[1]);
    return -1;
  }

  return 0;
}

static int xbee_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;
  strcpy(pgm->port, port);
  pinfo.serialinfo.baud = pgm->baudrate;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /* Wireless is lossier than normal serial */
  serial_recv_timeout = 1000;

  serdev = &xbee_serdev_frame;

  if (serial_open(port, pinfo, &pgm->fd) == -1) {
    return -1;
  }

  xbeedev_setresetpin(&pgm->fd, PDATA(pgm)->xbeeResetPin);

  /* Clear DTR and RTS */
  serial_set_dtr_rts(&pgm->fd, 0);
  usleep(250*1000);

  /* Set DTR and RTS back to high */
  serial_set_dtr_rts(&pgm->fd, 1);
  usleep(50*1000);

  /*
   * At this point stk500_drain() and stk500_getsync() calls would
   * normally be made.  But given that we have a transport layer over
   * the serial command stream, the drain and repeated STK_GET_SYNC
   * requests are not very helpful.  Instead, skip the draining
   * entirely, and issue the STK_GET_SYNC ourselves.
   */
  if (xbee_getsync(pgm) < 0)
    return -1;

  return 0;
}

static void xbee_close(PROGRAMMER *pgm)
{
  struct XBeeBootSession *xbs = xbeebootsession(&pgm->fd);

  /*
   * NB: This request is for the target device, not the locally
   * connected serial device.
   */
  serial_set_dtr_rts(&pgm->fd, 0);

  /*
   * We have tweaked a few settings on the XBee, including the RTS
   * mode and the reset pin's configuration.  Do a soft full reset,
   * restoring the device to its normal power-on settings.
   *
   * Note that this DOES mean that the remote XBee will be
   * uncontactable until it has restarted and re-established
   * communications on the mesh.
   */
  if (!xbs->directMode) {
    const int rc = sendAT(xbs, "AT FR", 'F', 'R', -1);
    xbeeATError(rc);
  }

  pmsg_notice("statistics for FRAME_LOCAL requests - %s->XBee(local)\n", progname);
  xbeeStatsSummarise(&xbs->groupSummary[XBEE_STATS_FRAME_LOCAL]);

  pmsg_notice("statistics for FRAME_REMOTE requests - %s->XBee(local)->XBee(target)\n", progname);
  xbeeStatsSummarise(&xbs->groupSummary[XBEE_STATS_FRAME_REMOTE]);

  pmsg_notice("statistics for TRANSMIT requests - %s->XBee(local)->XBee(target)->XBeeBoot\n", progname);
  xbeeStatsSummarise(&xbs->groupSummary[XBEE_STATS_TRANSMIT]);

  pmsg_notice("statistics for RECEIVE requests - XBeeBoot->XBee(target)->XBee(local)->%s\n", progname);
  xbeeStatsSummarise(&xbs->groupSummary[XBEE_STATS_RECEIVE]);

  xbeedev_free(xbs);

  pgm->fd.pfd = NULL;
}

static int xbee_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rc = 0;

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if (str_starts(extended_param, "xbeeresetpin=")) {
      int resetpin;
      if (sscanf(extended_param, "xbeeresetpin=%i", &resetpin) != 1 ||
          resetpin <= 0 || resetpin > 7) {
        pmsg_error("invalid xbeeresetpin '%s'\n", extended_param);
        rc = -1;
        continue;
      }

      PDATA(pgm)->xbeeResetPin = resetpin;
      continue;
    }
    if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xxbeeresetpin=<1..7> Set XBee pin DIO<1..7> as reset pin\n");
      msg_error("  -xhelp                Show this help menu and exit\n");
      exit(0);
    }

    pmsg_error("invalid extended parameter '%s'\n", extended_param);
    rc = -1;
  }

  return rc;
}

const char xbee_desc[] = "XBee Series 2 Over-The-Air (XBeeBoot)";

void xbee_initpgm(PROGRAMMER *pgm) {
  /*
   * This behaves like an Arduino, but with packet encapsulation of
   * the serial streams, XBee device management, and XBee GPIO for the
   * Auto-Reset feature. stk500.c sets PDATA(pgm)->xbeeResetPin
   */
  stk500_initpgm(pgm);

  strncpy(pgm->type, "XBee", sizeof(pgm->type));
  pgm->read_sig_bytes = xbee_read_sig_bytes;
  pgm->open = xbee_open;
  pgm->close = xbee_close;

  pgm->parseextparams = xbee_parseextparms;
}
