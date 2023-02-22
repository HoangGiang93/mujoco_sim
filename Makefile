all: mujoco

VERSION = 2.3.2
MUJOCO = mujoco-$(VERSION)-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/$(VERSION)/$(MUJOCO)
SOURCE_DIR = build/mujoco-$(VERSION)
SHA256SUM = cad8bf31cb06cfb590f80537c4adf3d32102a588bb1202774bc0efa1493f530d
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM) build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build