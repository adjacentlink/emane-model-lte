ACLOCAL_AMFLAGS = -I m4

SUBDIRS= \
  include/libemanelte \
  src                 \
  demo

edit = sed                                \
       -e 's|@VERSION[@]|$(VERSION)|g'    \
       -e 's|@DATE_RFC2822[@]|$(DATE_RFC2822)|g'

dist-hook:
	find $(distdir) -name "*.pb.*" -delete

cleantar:
	@(rm -f $(PACKAGE)*.tar.gz)

RPMARCH=`arch`

if HAVE_RPMBUILD

rpm: $(PACKAGE).spec dist-gzip
	mkdir -p .rpmbuild/BUILD \
	.rpmbuild/SPECS \
	.rpmbuild/SOURCES \
	.rpmbuild/SRPMS \
	.rpmbuild/RPMS/$(RPMARCH) \
	.rpmbuild/RPMS/noarch \
	.rpmbuild/tmp
	cp -f $(PACKAGE)-$(VERSION).tar.gz .rpmbuild/SOURCES
	rpmbuild --clean -ba $(top_srcdir)/$(PACKAGE).spec --target $(RPMARCH) \
        --define "_topdir $$PWD/.rpmbuild" \
        --define "_tmppath $$PWD/.rpmbuild/tmp"
	@echo "============================================================"
	@echo "RPMs located in .rpmbuild/RPMS and .rpmbuild/SRPMS"

endif

clean-local: cleantar cleandeb
	-rm -rf .rpmbuild
	-rm -rf .debbuild
	-rm -f $(PACKAGE).spec
	-rm -f debian/changelog
	-rm -f $(PACKAGE).pdf

$(PACKAGE).spec: $(PACKAGE).spec.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

if HAVE_DEB

deb: dist-gzip
	rm -rf .debbuild
	mkdir -p .debbuild
	cp -f $(PACKAGE)-$(VERSION).tar.gz .debbuild
	cd .debbuild && tar xzf $(PACKAGE)-$(VERSION).tar.gz
	$(edit) .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog.in > \
          .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog
	$(edit) .debbuild/$(PACKAGE)-$(VERSION)/debian/rules.in > \
          .debbuild/$(PACKAGE)-$(VERSION)/debian/rules
	chmod u+x .debbuild/$(PACKAGE)-$(VERSION)/debian/rules
	cd .debbuild/$(PACKAGE)-$(VERSION) && dpkg-source -b . && dpkg-buildpackage -F -us -uc
	@echo "============================================================"
	@echo "debs located in .debbuild"
cleandeb:
	dh_clean
else

cleandeb:

endif


install-exec-hook:
	$(mkinstalldirs) $(DESTDIR)$(datadir)/emane/manifest
	env LD_LIBRARY_PATH=src/.libs:$(LD_LIBRARY_PATH) src/emaneinfolte ue > \
	$(DESTDIR)$(datadir)/emane/manifest/lteueradiomodel.xml
	env LD_LIBRARY_PATH=src/.libs:$(LD_LIBRARY_PATH) src/emaneinfolte enb > \
	$(DESTDIR)$(datadir)/emane/manifest/lteenbradiomodel.xml

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/emane/manifest/lteenbradiomodel.xml
	rm -f $(DESTDIR)$(datadir)/emane/manifest/lteueradiomodel.xml



EXTRA_DIST =                   \
 AUTHORS                       \
 COPYING                       \
 ChangeLog                     \
 NEWS                          \
 $(PACKAGE).spec.in            \
 debian                        \
 autogen.sh

DISTCLEANFILES = .rpmbuild
