EXTRA_DIST=           \
  two_ues             \
  templates

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/demo
	cp -R two_ues $(DESTDIR)$(datadir)/$(PACKAGE)/demo
	cp -R templates $(DESTDIR)$(datadir)/$(PACKAGE)/demo

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/demo/two_ues/*
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo/two_ues
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/enb/*
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/enb
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/epc/*
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/epc
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/host/*
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/host
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/ue/*
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates/ue
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo/templates
	rmdir $(DESTDIR)$(datadir)/$(PACKAGE)/demo
