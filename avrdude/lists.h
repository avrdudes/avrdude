/*
 * Copyright (c) 1990, 1991, 1992, 1993, 1994, 1995, 1996, 1997,
 *   1998, 1999, 2000, 2001, 2002  Brian S. Dean <bsd@bsdhome.com>
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BRIAN S. DEAN ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BRIAN S. DEAN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * 
 */

/* $Id$ */

/*----------------------------------------------------------------------
  Id: lists.h,v 1.2 2001/08/19 23:13:17 bsd Exp $
  ----------------------------------------------------------------------*/
/*----------------------------------------------------------------------
  General purpose linked list routines - header file declarations.

  Author : Brian Dean
  Date   : 10 January, 1990
  ----------------------------------------------------------------------*/
#ifndef __lists_h__
#define __lists_h__

#include <stdio.h>

typedef void * LISTID;
typedef void * LNODEID;


/*----------------------------------------------------------------------
  several defines to access the LIST structure as as stack or a queue
  --- use for program readability
  ----------------------------------------------------------------------*/
#define STACKID LISTID
#define SNODEID LNODEID
#define QUEUEID LISTID
#define QNODEID LNODEID


#define PUSH(s,d)    lins_n(s,d,1)   /* push 'd' onto the stack */
#define POP(s)       lrmv_n(s,1)     /* pop the stack */
#define LOOKSTACK(s) lget_n(s,1)     /* look at the top of the stack, 
					but don't pop */


#define ENQUEUE(q,d) lins_n(q,d,1)   /* put 'd' on the end of the queue */
#define DEQUEUE(q)   lrmv(q)         /* remove next item from the front of 
					the queue */
#define REQUEUE(q,d) ladd(q,d)       /* re-insert (push) item back on the
					front of the queue */
#define LOOKQUEUE(q) lget(q)         /* return next item on the queue, 
					but don't dequeue */
#define QUEUELEN(q)  lsize(q)       /* length of the queue */


#define LISTADD(l,d) ladd(l,d)       /* add to end of the list */
#define LISTRMV(l,d) lrmv_d(l,d)     /* remove from end of the list */


#define LISTSZ  32  /* size of internal private LIST structure */


/* .................... Function Prototypes .................... */

LISTID     lcreat      ( void * liststruct, int poolsize );
void       ldestroy    ( LISTID lid );
void       ldestroy_cb ( LISTID lid, void (*ucleanup)() );

LNODEID    lfirst ( LISTID  ); /* head of the list */
LNODEID    llast  ( LISTID  ); /* tail of the list */
LNODEID    lnext  ( LNODEID ); /* next item in the list */
LNODEID    lprev  ( LNODEID ); /* previous item in the list */
void     * ldata  ( LNODEID ); /* data at the current position */
int        lsize  ( LISTID  ); /* number of elements in the list */

int        ladd     ( LISTID lid, void * p );
int        laddo    ( LISTID lid, void *p, 
		      int (*compare)(const void *p1,const void *p2),
		      LNODEID * firstdup );
int        laddu    ( LISTID lid, void * p, 
		      int (*compare)(const void *p1,const void *p2));
int        lins_n   ( LISTID lid, void * d, unsigned int n );
int        lins_ln  ( LISTID lid, LNODEID lnid, void * data_ptr );

void     * lget    ( LISTID lid );
void     * lget_n  ( LISTID lid, unsigned int n );
LNODEID    lget_ln ( LISTID lid, unsigned int n );

void     * lrmv    ( LISTID lid );
void     * lrmv_n  ( LISTID lid, unsigned int n );
void     * lrmv_ln ( LISTID lid, LNODEID lnid );
void     * lrmv_d  ( LISTID lid, void * data_ptr );

LISTID     lcat    ( LISTID lid1, LISTID lid2 );

void     * lsrch   ( LISTID lid, void * p, int (*compare)(void *p1,void *p2));

int        lprint  ( FILE * f, LISTID lid );

#endif
