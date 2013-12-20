/** vim: set filetype=Makefile ts=2 sts=2 sw=2 fdm=indent */

define my-dir
$(strip \
  $(eval LOCAL_MODULE_MAKEFILE := $$(lastword $$(MAKEFILE_LIST))) \
  $(if $(filter $(CLEAR_VARS),$(LOCAL_MODULE_MAKEFILE)), \
    $(error LOCAL_PATH must be set before including $$(CLEAR_VARS)) \
   , \
    $(patsubst %/,%,$(dir $(LOCAL_MODULE_MAKEFILE))) \
   ) \
 )
endef

define all-subdir-makefiles
$(call all-makefiles-under,$(call my-dir))
endef

define all-subdir-makefiles
$(call all-makefiles-under,$(call my-dir))
endef

define all-named-subdir-makefiles
$(wildcard $(addsuffix /Android.mk, $(addprefix $(call my-dir)/,$(1))))
endef

define all-c-files-under
$(patsubst ./%,%, \
	$(shell find $(1) -name "*.c" -and -not -name ".*") \
 )
endef

define all-subdir-c-files
$(call all-c-files-under,.)
endef
