#!/usr/bin/env bash
set -e
set -x

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

BUILD_DIR=${BUILD_DIR:-/data/openpilot}
SOURCE_DIR="$(git rev-parse --show-toplevel)"

export PYTHONPATH="$BUILD_DIR:$BUILD_DIR/msgq_repo:$BUILD_DIR/opendbc_repo:$BUILD_DIR/rednose_repo:$BUILD_DIR/teleoprtc_repo:$BUILD_DIR/tinygrad_repo"

if [ -z "$RELEASE_BRANCH" ] && [ "$SKIP_PUSH" != "1" ]; then
  echo "RELEASE_BRANCH is not set"
  exit 1
fi

BUILD_BRANCH=${BUILD_BRANCH:-release-mici-staging}

SCONS=${SCONS:-$(command -v scons || true)}
if [ -z "$SCONS" ] && [ -x /usr/local/venv/bin/scons ]; then
  SCONS=/usr/local/venv/bin/scons
fi
if [ -z "$SCONS" ]; then
  echo "scons executable not found"
  exit 1
fi
export PATH="$(dirname "$SCONS"):$PATH"

case "$(readlink -f "$BUILD_DIR")" in
  /|/data|/data/sp|"$(readlink -f "$SOURCE_DIR")")
    echo "Unsafe BUILD_DIR: $BUILD_DIR"
    exit 1
    ;;
esac


# set git identity
source $DIR/identity.sh

echo "[-] Setting up repo T=$SECONDS"
if ! git -C "$SOURCE_DIR" worktree remove --force "$BUILD_DIR" 2>/dev/null; then
  rm -rf $BUILD_DIR
fi
git -C "$SOURCE_DIR" worktree prune
git -C "$SOURCE_DIR" worktree add --detach --no-checkout "$BUILD_DIR"
cd $BUILD_DIR
git update-ref -d "refs/heads/$BUILD_BRANCH"
git symbolic-ref HEAD "refs/heads/$BUILD_BRANCH"
git read-tree --empty

# do the files copy
echo "[-] copying files T=$SECONDS"
cd $SOURCE_DIR
./tools/release/release_files.py | xargs -0 cp -pR --parents -t "$BUILD_DIR" --

# in the directory
cd $BUILD_DIR

# use the full CPU available for speeding up the build.
# openpilot resets the CPU frequencies when test_onroad.py runs below.
for policy in /sys/devices/system/cpu/cpufreq/policy*; do
  [ -d "$policy" ] || continue
  hardware_max="$(cat "$policy/cpuinfo_max_freq")"
  if ! echo "$hardware_max" | sudo tee "$policy/scaling_max_freq" >/dev/null; then
    echo "Warning: $policy rejected scaling_max_freq=$hardware_max; keeping the kernel-selected limit"
  fi
done

"$SCONS"
python3 tools/release/check_model_artifacts.py
if [ -n "$INCLUDE_BIG_MODEL" ]; then
  test -f openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl.chunkmanifest
fi

if [ -z "$PANDA_DEBUG_BUILD" ]; then
  # release panda fw
  CERT=/data/pandaextra/certs/release RELEASE=1 "$SCONS" panda/
else
  # build with ALLOW_DEBUG=1 to enable features like experimental longitudinal
  "$SCONS" panda/
fi

# Ensure the flattened release has neither submodule metadata nor gitlinks.
if [ -f .gitmodules ] || git ls-files -s | awk '$1 == "160000" { found = 1 } END { exit !found }'; then
  echo "submodule metadata or gitlinks found in release"
  exit 1
fi

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name '__pycache__' -delete
rm -rf .sconsign.dblite Jenkinsfile tools/release/
rm -f openpilot/selfdrive/modeld/models/*.onnx*
rm -f openpilot/sunnypilot/modeld*/models/*.onnx*

find openpilot/third_party/ -name '*x86*' -exec rm -r {} +
find openpilot/third_party/ -name '*Darwin*' -exec rm -r {} +

# Mark as prebuilt release
touch prebuilt

# Preserve the exact source anchor in the orphan prebuild snapshot.
git -C "$SOURCE_DIR" rev-parse HEAD > git_src_commit
git -C "$SOURCE_DIR" show -s --format='%ct %ci' HEAD > git_src_commit_date

VERSION=$(cat openpilot/sunnypilot/common/version.h | awk -F[\"-]  '{print $2}')
# Add built files to git
# writing larger objects is faster than compressing them on-device
git -c core.compression=0 add -f .
git -c core.compression=0 -c gc.auto=0 commit -m "openpilot v$VERSION"

# Run tests
cd $BUILD_DIR
RELEASE=1 ./openpilot/selfdrive/test/test_onroad.py
#tools/test_runner.py openpilot/selfdrive/car/tests/test_car_interfaces.py

echo "[-] pushing release T=$SECONDS"
if [ "$SKIP_PUSH" = "1" ]; then
  echo "[-] SKIP_PUSH=1; prebuild commit is ready at $BUILD_DIR"
  exit 0
fi
REFS=()
for branch in ${RELEASE_BRANCH//,/ }; do
  REFS+=("$BUILD_BRANCH:$branch")
done
# uploading the larger pack is faster than spending CPU to optimize it
git -c pack.window=0 -c pack.depth=0 -c pack.compression=0 push -f origin "${REFS[@]}"

echo "[-] done T=$SECONDS"
