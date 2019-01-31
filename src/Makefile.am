pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libemanelte.pc

lib_LTLIBRARIES = libemanelte.la

libemanelte_la_CPPFLAGS=   \
 -I@top_srcdir@/include \
 $(AM_CFLAGS)              \
 $(libemane_CFLAGS)        \
 $(libostatistic_CFLAGS)

libemanelte_la_SOURCES =       \
 configmanager.cc              \
 downlinkresourcegridparams.cc \
 downlinksinrtesterimpl.cc     \
 enbradiostatisticmanager.cc   \
 enbmessageprocessor.cc        \
 enbstatisticmanager.cc        \
 epcstatisticmanager.cc        \
 frequencysegmentbuilder.cc    \
 manifest.cc                   \
 mbmsstatisticmanager.cc       \
 mhalcommon.cc                 \
 mhalenb.cc                    \
 mhalenb_impl.cc               \
 mhalue.cc                     \
 mhalue_impl.cc                \
 pormanager.cc                 \
 radiostatisticmanager.cc      \
 pendingmessages.cc            \
 readymessages.cc              \
 sinrtester.cc                 \
 statisticmanager.cc           \
 uemessageprocessor.cc         \
 uestatisticmanager.cc         \
 ueradiostatisticmanager.cc    \
 uplinkresourcegridparams.cc   \
 uplinksinrtesterimpl.cc       \
 utils.cc                      \
 xmlvalidator.cc

nodist_libemanelte_la_SOURCES = \
 enbotamessage.pb.cc            \
 ueotamessage.pb.cc             \
 otacommon.pb.cc                \
 txcontrolmessage.pb.cc

EXTRA_DIST=                     \
 basicstatistichelper.h         \
 channelcounter.h               \
 commonresourcegridparams.h     \
 configmanager.h                \
 downlinkresourcegridparams.h   \
 downlinksinrtesterimpl.h       \
 enbotamessage.proto            \
 enbmessageprocessor.h          \
 enbradiostatisticmanager.h     \
 frequencysegmentbuilder.h      \
 gtpcinterfacetypes.h           \
 iptrafficstats.h               \
 ltedefs.h                      \
 manifest.h                     \
 mhalenb_impl.h                 \
 mhalcommon.h                   \
 mhalphy.h                      \
 mhalue_impl.h                  \
 otacommon.proto                \
 pendingmessages.h              \
 pormanager.h                   \
 radiomodel.h                   \
 radiomodel.inl                 \
 radiostatisticmanager.h        \
 readymessages.h                \
 segmentmap.h                   \
 sinrtesterimpl.h               \
 spectrumwindowcache.h          \
 stateinfo.h                    \
 statisticmanager.h             \
 statistichelper.h              \
 timinginfo.h                   \
 txcontrolmessage.proto         \
 uemessageprocessor.h           \
 ueradiostatisticmanager.h      \
 ueotamessage.proto             \
 uplinkresourcegridparams.h     \
 uplinksinrtesterimpl.h         \
 utils.h                        \
 xmlvalidator.h

BUILT_SOURCES =        \
 $(nodist_libemanelte_la_SOURCES)

libemanelte_la_LDFLAGS=       \
 $(LDFLAGS)                   \
 $(libemane_LIBS)             \
 $(libostatistic_LIBS)        \
 -avoid-version

edit = sed \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

# generate code from proto files and adjust includes for relocation of headers to include/libemanelte
%.pb.cc: %.proto
	protoc -I=. --cpp_out=. $<
	sed -i -e 's|$(*).pb.h|libemanelte/$(*).pb.h|g' $@
	rm -f $(*).pb.h

clean-local:
	rm -f $(BUILT_SOURCES)

noinst_PROGRAMS = emaneinfolte

emaneinfolte_SOURCES =  \
 emaneinfolte.cc

emaneinfolte_CPPFLAGS =                  \
 -I@top_srcdir@/include                  \
 -I@top_srcdir@/src/libemane             \
 $(AM_CPPFLAGS)                          \
 $(libemane_CFLAGS)

emaneinfolte_LDADD = \
 $(libemane_LIBS)   \
 $(libuuid_LIBS)    \
 libemanelte.la
