all: mujoco

VERSION = 2.3.1
MUJOCO = mujoco-$(VERSION)-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/$(VERSION)/$(MUJOCO)
SOURCE_DIR = build/mujoco-$(VERSION)
SHA256SUM = 70efb3be950674913703e594b4c4b70f5bbb7e85f89a58ebb519dfde82066652
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM)  build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build