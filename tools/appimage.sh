#!/usr/bin/env bash
# Builds target/appimage/MARV-GCS-x86_64.AppImage: the station window (release build) as one file that runs
# on x86_64 Linux with glibc 2.39 or newer (the build machine's; Ubuntu 24.04). Usage: tools/appimage.sh
#
# The AppImage is the AppImage project's type-2 runtime (pinned release, checked by SHA-256) followed by a
# squashfs of an AppDir: AppRun, the desktop entry, the icon and usr/bin/marv-gcs. Nothing else is bundled:
# libc, libudev and libcap, and the display libraries the window loads at run time (X11 or Wayland, OpenGL),
# come from the host, as on every desktop Linux. AppRun starts the window in ~/Documents/MARV-GCS, so its
# recordings and downloaded logs go to ~/Documents/MARV-GCS/logs whatever folder the AppImage is started from.
set -euo pipefail
cd "$(dirname "$0")/.."

runtime_url="https://github.com/AppImage/type2-runtime/releases/download/20251108/runtime-x86_64"
runtime_sha256="2fca8b443c92510f1483a883f60061ad09b46b978b2631c807cd873a47ec260d"
out="target/appimage"
runtime="$out/runtime-x86_64"
image="$out/MARV-GCS-x86_64.AppImage"

mkdir -p "$out"
if ! { [ -f "$runtime" ] && echo "$runtime_sha256  $runtime" | sha256sum -c --quiet --status; }; then
    curl -sSfL --max-time 120 -o "$runtime.part" "$runtime_url"
    echo "$runtime_sha256  $runtime.part" | sha256sum -c --quiet
    mv "$runtime.part" "$runtime"
fi

cargo build --release -p marv-gcs

appdir="$out/AppDir"
rm -rf "$appdir"
mkdir -p "$appdir/usr/bin"
cp target/release/marv-gcs "$appdir/usr/bin/marv-gcs"
cp packaging/marv-gcs.desktop packaging/marv-gcs.svg "$appdir/"
ln -s marv-gcs.svg "$appdir/.DirIcon"
cat > "$appdir/AppRun" <<'EOF'
#!/bin/sh
here="${APPDIR:-$(dirname "$(readlink -f "$0")")}"
mkdir -p "$HOME/Documents/MARV-GCS" && cd "$HOME/Documents/MARV-GCS" || exit 1
exec "$here/usr/bin/marv-gcs" "$@"
EOF
chmod +x "$appdir/AppRun"
desktop-file-validate "$appdir/marv-gcs.desktop"

rm -f "$out/marv-gcs.squashfs" "$image"
mksquashfs "$appdir" "$out/marv-gcs.squashfs" -root-owned -noappend -quiet -comp zstd
cat "$runtime" "$out/marv-gcs.squashfs" > "$image"
chmod +x "$image"
rm -f "$out/marv-gcs.squashfs"
echo "$image ($(du -h "$image" | cut -f1))"
