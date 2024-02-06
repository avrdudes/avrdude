BUILD_SCRIPT_DIR = build-helpers

# Check that package version matches git version before creating dist tarballs
dist-hook: cur-version-check cur-version-stamp
distcheck-hook: cur-version-check

# Note: We cannot run autoreconf from here, because we would need some way to
#       restart the whole dist process from the start and there is none.
EXTRA_DIST += $(top_srcdir)/$(BUILD_SCRIPT_DIR)/package-version
cur-version-check:
	@cur_ver=`$(top_srcdir)/$(BUILD_SCRIPT_DIR)/package-version $(top_srcdir) version-stamp`; \
	if test "x$${cur_ver}" = "x$(PACKAGE_VERSION)"; then :; else \
		echo "ERROR: Recorded PACKAGE_VERSION and current version do not match:"; \
		echo "         current  version:         $${cur_ver}"; \
		echo "         recorded PACKAGE_VERSION: $(PACKAGE_VERSION)"; \
		rm -rf "$(top_srcdir)/autom4te.cache"; \
		echo "Update PACKAGE_VERSION by running autoreconf(1)."; \
		exit 1; \
	fi

# Version stamp files can only exist in tarball source trees.
#
# So there is no need to generate them anywhere else or to clean them
# up anywhere.
cur-version-stamp:
	echo "$(PACKAGE_VERSION)" > "$(distdir)/version-stamp"
