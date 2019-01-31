libemanelteincdir = $(includedir)/libemanelte/

libemanelteinc_HEADERS = \
 mhal.h                  \
 mhalconfig.h            \
 mhalenb.h               \
 mhalue.h                \
 enbstatisticmanager.h   \
 epcstatisticmanager.h   \
 mbmsstatisticmanager.h  \
 sinrtester.h            \
 uestatisticmanager.h

nodist_libemanelteinc_HEADERS = \
 enbotamessage.pb.h      \
 otacommon.pb.h          \
 txcontrolmessage.pb.h   \
 ueotamessage.pb.h

BUILT_SOURCES= \
 $(nodist_libemanelteinc_HEADERS)

install-exec-hook:
	$(mkinstalldirs) $(DESTDIR)$(libemanelteincdir)

clean-local:
	rm -f $(BUILT_SOURCES)

%.pb.h:
	protoc -I=../../src --cpp_out=. ../../src/$(*).proto
	sed -i -e 's|otacommon.pb.h|libemanelte/otacommon.pb.h|g' $@
	rm -f $(*).pb.cc
