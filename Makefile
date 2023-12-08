#############################################################################
# Makefile for building: bin/sim
# Generated by qmake (3.1) (Qt 5.9.2)
# Project:  Simulation.pro
# Template: app
# Command: /home/xiaohan/Qt5.9.2/5.9.2/gcc_64/bin/qmake -o Makefile Simulation.pro
#############################################################################

MAKEFILE      = Makefile

first: release
install: release-install
uninstall: release-uninstall
QMAKE         = /home/xiaohan/Qt5.9.2/5.9.2/gcc_64/bin/qmake
DEL_FILE      = rm -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p
COPY          = cp -f
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
INSTALL_FILE  = install -m 644 -p
INSTALL_PROGRAM = install -m 755 -p
INSTALL_DIR   = cp -f -R
QINSTALL      = /home/xiaohan/Qt5.9.2/5.9.2/gcc_64/bin/qmake -install qinstall
QINSTALL_PROGRAM = /home/xiaohan/Qt5.9.2/5.9.2/gcc_64/bin/qmake -install qinstall -exe
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
TAR           = tar -cf
COMPRESS      = gzip -9f
DISTNAME      = sim1.0.0
DISTDIR = /home/xiaohan/Simulation_Kinematics/LateralControl/build/sim1.0.0
SUBTARGETS    =  \
		release \
		debug


release: FORCE
	$(MAKE) -f $(MAKEFILE).Release
release-make_first: FORCE
	$(MAKE) -f $(MAKEFILE).Release 
release-all: FORCE
	$(MAKE) -f $(MAKEFILE).Release all
release-clean: FORCE
	$(MAKE) -f $(MAKEFILE).Release clean
release-distclean: FORCE
	$(MAKE) -f $(MAKEFILE).Release distclean
release-install: FORCE
	$(MAKE) -f $(MAKEFILE).Release install
release-uninstall: FORCE
	$(MAKE) -f $(MAKEFILE).Release uninstall
debug: FORCE
	$(MAKE) -f $(MAKEFILE).Debug
debug-make_first: FORCE
	$(MAKE) -f $(MAKEFILE).Debug 
debug-all: FORCE
	$(MAKE) -f $(MAKEFILE).Debug all
debug-clean: FORCE
	$(MAKE) -f $(MAKEFILE).Debug clean
debug-distclean: FORCE
	$(MAKE) -f $(MAKEFILE).Debug distclean
debug-install: FORCE
	$(MAKE) -f $(MAKEFILE).Debug install
debug-uninstall: FORCE
	$(MAKE) -f $(MAKEFILE).Debug uninstall

Makefile: Simulation.pro ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/linux-g++/qmake.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/spec_pre.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/unix.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/linux.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/sanitize.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/gcc-base.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/gcc-base-unix.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/g++-base.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/g++-unix.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/qconfig.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3danimation.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3danimation_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dcore.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dcore_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dextras.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dextras_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dinput.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dinput_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dlogic.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dlogic_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquick.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquick_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickanimation.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickanimation_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickextras.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickextras_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickinput.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickinput_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickrender.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickrender_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickscene2d.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickscene2d_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3drender.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3drender_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_accessibility_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bluetooth.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bluetooth_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bootstrap_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_concurrent.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_concurrent_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_core.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_core_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_dbus.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_dbus_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designer.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designer_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designercomponents_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_devicediscovery_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_egl_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_eglfsdeviceintegration_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_eventdispatcher_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_fb_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_fontdatabase_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gamepad.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gamepad_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_glx_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gui.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gui_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_help.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_help_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_input_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_kms_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_linuxaccessibility_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_location.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_location_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimedia.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimedia_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimediawidgets.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimediawidgets_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_network.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_network_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_nfc.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_nfc_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_opengl.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_opengl_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_openglextensions.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_openglextensions_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_packetprotocol_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_platformcompositor_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_positioning.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_positioning_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_printsupport.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_printsupport_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qml.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qml_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmldebug_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmldevtools_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmltest.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmltest_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qtmultimediaquicktools_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quick.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quick_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickcontrols2.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickcontrols2_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickparticles_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quicktemplates2_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickwidgets.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickwidgets_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_scxml.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_scxml_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sensors.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sensors_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialbus.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialbus_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialport.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialport_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_service_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sql.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sql_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_svg.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_svg_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_testlib.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_testlib_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_theme_support_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uiplugin.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uitools.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uitools_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webchannel.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webchannel_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_websockets.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_websockets_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webview.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webview_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_widgets.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_widgets_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_x11extras.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_x11extras_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xml.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xml_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xmlpatterns.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xmlpatterns_private.pri \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt_functions.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt_config.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/linux-g++/qmake.conf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/spec_post.prf \
		.qmake.stash \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exclusive_builds.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/toolchain.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/default_pre.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/resolve_config.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exclusive_builds_post.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/default_post.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/warn_on.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/resources.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/moc.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/unix/thread.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qmake_use.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/file_copies.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/testcase_targets.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exceptions.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/yacc.prf \
		../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/lex.prf \
		Simulation.pro \
		../../Qt5.9.2/5.9.2/gcc_64/lib/libQt5Core.prl
	$(QMAKE) -o Makefile Simulation.pro
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/spec_pre.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/unix.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/linux.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/sanitize.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/gcc-base.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/gcc-base-unix.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/g++-base.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/g++-unix.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/qconfig.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3danimation.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3danimation_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dcore.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dcore_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dextras.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dextras_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dinput.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dinput_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dlogic.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dlogic_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquick.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquick_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickanimation.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickanimation_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickextras.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickextras_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickinput.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickinput_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickrender.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickrender_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickscene2d.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickscene2d_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3drender.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3drender_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_accessibility_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bluetooth.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bluetooth_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bootstrap_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_concurrent.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_concurrent_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_core.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_core_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_dbus.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_dbus_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designer.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designer_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designercomponents_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_devicediscovery_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_egl_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_eglfsdeviceintegration_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_eventdispatcher_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_fb_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_fontdatabase_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gamepad.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gamepad_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_glx_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gui.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gui_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_help.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_help_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_input_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_kms_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_linuxaccessibility_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_location.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_location_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimedia.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimedia_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimediawidgets.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimediawidgets_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_network.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_network_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_nfc.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_nfc_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_opengl.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_opengl_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_openglextensions.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_openglextensions_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_packetprotocol_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_platformcompositor_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_positioning.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_positioning_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_printsupport.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_printsupport_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qml.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qml_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmldebug_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmldevtools_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmltest.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmltest_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qtmultimediaquicktools_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quick.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quick_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickcontrols2.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickcontrols2_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickparticles_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quicktemplates2_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickwidgets.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickwidgets_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_scxml.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_scxml_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sensors.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sensors_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialbus.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialbus_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialport.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialport_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_service_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sql.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sql_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_svg.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_svg_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_testlib.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_testlib_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_theme_support_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uiplugin.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uitools.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uitools_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webchannel.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webchannel_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_websockets.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_websockets_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webview.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webview_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_widgets.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_widgets_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_x11extras.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_x11extras_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xml.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xml_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xmlpatterns.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xmlpatterns_private.pri:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt_functions.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt_config.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/linux-g++/qmake.conf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/spec_post.prf:
.qmake.stash:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exclusive_builds.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/toolchain.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/default_pre.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/resolve_config.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exclusive_builds_post.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/default_post.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/warn_on.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/resources.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/moc.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/unix/thread.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qmake_use.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/file_copies.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/testcase_targets.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exceptions.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/yacc.prf:
../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/lex.prf:
Simulation.pro:
../../Qt5.9.2/5.9.2/gcc_64/lib/libQt5Core.prl:
qmake: FORCE
	@$(QMAKE) -o Makefile Simulation.pro

qmake_all: FORCE

make_first: release-make_first debug-make_first  FORCE
all: release-all debug-all  FORCE
clean: release-clean debug-clean  FORCE
distclean: release-distclean debug-distclean  FORCE
	-$(DEL_FILE) Makefile
	-$(DEL_FILE) .qmake.stash

release-mocclean:
	$(MAKE) -f $(MAKEFILE).Release mocclean
debug-mocclean:
	$(MAKE) -f $(MAKEFILE).Debug mocclean
mocclean: release-mocclean debug-mocclean

release-mocables:
	$(MAKE) -f $(MAKEFILE).Release mocables
debug-mocables:
	$(MAKE) -f $(MAKEFILE).Debug mocables
mocables: release-mocables debug-mocables

check: first

benchmark: first
FORCE:

dist: distdir FORCE
	(cd `dirname $(DISTDIR)` && $(TAR) $(DISTNAME).tar $(DISTNAME) && $(COMPRESS) $(DISTNAME).tar) && $(MOVE) `dirname $(DISTDIR)`/$(DISTNAME).tar.gz . && $(DEL_FILE) -r $(DISTDIR)

distdir: release-distdir debug-distdir FORCE
	@test -d $(DISTDIR) || mkdir -p $(DISTDIR)
	$(COPY_FILE) --parents ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/spec_pre.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/unix.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/linux.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/sanitize.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/gcc-base.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/gcc-base-unix.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/g++-base.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/common/g++-unix.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/qconfig.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3danimation.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3danimation_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dcore.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dcore_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dextras.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dextras_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dinput.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dinput_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dlogic.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dlogic_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquick.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquick_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickanimation.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickanimation_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickextras.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickextras_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickinput.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickinput_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickrender.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickrender_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickscene2d.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3dquickscene2d_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3drender.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_3drender_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_accessibility_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bluetooth.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bluetooth_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_bootstrap_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_concurrent.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_concurrent_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_core.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_core_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_dbus.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_dbus_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designer.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designer_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_designercomponents_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_devicediscovery_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_egl_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_eglfsdeviceintegration_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_eventdispatcher_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_fb_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_fontdatabase_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gamepad.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gamepad_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_glx_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gui.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_gui_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_help.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_help_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_input_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_kms_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_linuxaccessibility_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_location.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_location_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimedia.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimedia_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimediawidgets.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_multimediawidgets_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_network.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_network_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_nfc.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_nfc_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_opengl.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_opengl_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_openglextensions.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_openglextensions_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_packetprotocol_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_platformcompositor_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_positioning.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_positioning_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_printsupport.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_printsupport_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qml.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qml_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmldebug_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmldevtools_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmltest.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qmltest_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_qtmultimediaquicktools_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quick.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quick_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickcontrols2.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickcontrols2_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickparticles_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quicktemplates2_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickwidgets.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_quickwidgets_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_scxml.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_scxml_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sensors.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sensors_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialbus.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialbus_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialport.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_serialport_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_service_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sql.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_sql_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_svg.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_svg_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_testlib.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_testlib_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_theme_support_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uiplugin.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uitools.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_uitools_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webchannel.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webchannel_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_websockets.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_websockets_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webview.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_webview_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_widgets.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_widgets_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_x11extras.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_x11extras_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xml.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xml_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xmlpatterns.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/modules/qt_lib_xmlpatterns_private.pri ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt_functions.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt_config.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/linux-g++/qmake.conf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/spec_post.prf .qmake.stash ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exclusive_builds.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/toolchain.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/default_pre.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/resolve_config.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exclusive_builds_post.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/default_post.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/warn_on.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qt.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/resources.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/moc.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/unix/thread.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/qmake_use.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/file_copies.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/testcase_targets.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/exceptions.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/yacc.prf ../../Qt5.9.2/5.9.2/gcc_64/mkspecs/features/lex.prf Simulation.pro $(DISTDIR)/

release-distdir: FORCE
	$(MAKE) -e -f $(MAKEFILE).Release distdir DISTDIR=$(DISTDIR)/

debug-distdir: FORCE
	$(MAKE) -e -f $(MAKEFILE).Debug distdir DISTDIR=$(DISTDIR)/

$(MAKEFILE).Release: Makefile
$(MAKEFILE).Debug: Makefile
