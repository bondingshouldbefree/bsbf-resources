include $(TOPDIR)/rules.mk

PKG_NAME:=bsbf-mptcp
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/bsbf-mptcp
  SECTION:=net
  CATEGORY:=Network
  TITLE:=bsbf-mptcp
endef

define Build/Compile
endef

define Package/bsbf-mptcp/install
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/etc/config/bsbf-mptcp $(1)/etc/config

	$(INSTALL_DIR) $(1)/etc/hotplug.d/iface
	$(INSTALL_BIN) ./files/etc/hotplug.d/iface/99-bsbf-mptcp $(1)/etc/hotplug.d/iface

	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/etc/init.d/bsbf-mptcp $(1)/etc/init.d

	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) ./files/usr/sbin/bsbf-mptcp $(1)/usr/sbin
	$(INSTALL_BIN) ./files/usr/sbin/bsbf-mptcp-helper $(1)/usr/sbin
endef

$(eval $(call BuildPackage,bsbf-mptcp))
