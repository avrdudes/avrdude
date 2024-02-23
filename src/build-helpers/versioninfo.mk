# -*- makefile-automake -*-

EXTRA_DIST += build-helpers/versioninfo.m4
EXTRA_DIST += build-helpers/versioninfo.md
EXTRA_DIST += build-helpers/versioninfo.sh

# Before creating dist tarballs, check that autom4te version matches
# versioninfo script version.
dist-hook: versioninfo-check versioninfo-stamp
distcheck-hook: versioninfo-check

# Note: We cannot run autoreconf from this make recipe, because we would
#       need some way to restart the whole dist process from the start
#       and there is none.
versioninfo-check:
	@:; \
	$(top_srcdir)/build-helpers/versioninfo.sh "$(top_srcdir)" "@VERSIONINFO_STAMPFILE@" \
	| ( @VERSIONINFO_READ@; \
	    if @VERSIONINFO_IS_UNCHANGED@ \
	      exit 0; \
	    fi; \
	    rm -rf "$(top_srcdir)/autom4te.cache"; \
	    echo "Update the recorded version information by re-running bootstrap/autoreconf(1)."; \
	    exit 1; )

# Version stamp files can only exist in tarball source trees.
#
# So there is no need to generate them anywhere else or to clean them
# up anywhere.
versioninfo-stamp:
	@VERSIONINFO_WRITE@ > "$(distdir)/versioninfo-stamp"
