all: mujoco

MUJOCO = mujoco-2.2.0-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/2.2.0/$(MUJOCO)
SOURCE_DIR = build/mujoco-2.2.0
SHA256SUM = b1d1b0f56f34808159d0384c750f60235eb9f5682e4d2f61d117c1be9ade2959
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM)  build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build