include $(TOPDIR)/rules.mk

PKG_NAME:=shoes
PKG_VERSION:=0.2.7
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-v$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=https://codeload.github.com/cfal/shoes/tar.gz/v$(PKG_VERSION)?
PKG_HASH:=8d2e63a021f58e09e827f9ec7d606a0583b578c55cad09619d6b7705ccd3a400

PKG_BUILD_DEPENDS:=rust/host
PKG_BUILD_PARALLEL:=1

# On x86_64 release build is 15M, with opt-level=z 11M, with opt-level=z and
# codegen-units=1 9.9M.
RUSTFLAGS += -C opt-level=z -C codegen-units=1

include $(INCLUDE_DIR)/package.mk
include ../../packages/lang/rust/rust-package.mk

define Package/shoes
  SECTION:=net
  CATEGORY:=Network
  TITLE:=shoes
endef

define Package/shoes/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/bin/shoes $(1)/usr/bin/
endef

$(eval $(call RustBinPackage,shoes))
$(eval $(call BuildPackage,shoes))
