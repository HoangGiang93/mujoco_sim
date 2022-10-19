all: mujoco

VERSION = 2.3.0
MUJOCO = mujoco-$(VERSION)-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/$(VERSION)/$(MUJOCO)
SOURCE_DIR = build/mujoco-$(VERSION)
SHA256SUM = 4f8ce72dfba6e97a5b938e9094b2724f84e1f4692ff299e7400bbd4feb3a5b49
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM)  build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build