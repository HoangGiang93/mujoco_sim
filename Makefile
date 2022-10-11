all: mujoco

MUJOCO = mujoco-2.2.2-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/2.2.2/$(MUJOCO)
SOURCE_DIR = build/mujoco-2.2.2
SHA256SUM = c887a4c315201ce47cf654e0115a55c29648567450d44cd8afe8e3a09d34ea1e
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM)  build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build