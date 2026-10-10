#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly SDK_ROOT="${PACKAGE_DIR}/SDK"
readonly ROS_CONFIG_FILE="${PACKAGE_DIR}/config/OrbbecSDKConfig_v2.0.xml"

dry_run=false
single_arch=false
work_dir=""
arm64_backup_dir=""
x64_backup_dir=""
config_backup_file=""

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--dry-run] [--single-arch] <arm64-or-x86_64-sdk-directory-or.tar.gz>

Replace the supplied architecture and, when found, its paired SDK.
First try swapping "_arm64" and "_x86_64" (before .tar.gz for archives).
Then match the same version and commit with a different numeric build timestamp.
The pair must use the same input format. Missing or ambiguous pairs produce a
warning and only the supplied architecture is updated.
Use --single-arch to update only the supplied architecture without searching.
The shared runtime XML is updated from the supplied SDK, also in single-arch mode.
Archives are extracted into temporary directories and cleaned up on exit.

Only the files used or distributed by orbbec_camera are copied:
  - include/libobsensor/
  - lib/libOrbbecSDK.so*
  - lib/OrbbecSDKConfig-release.cmake
  - lib/OrbbecSDKConfig.cmake
  - lib/OrbbecSDKVersion.cmake
  - lib/extensions/
  - lib/OrbbecSDKConfig.xml -> config/OrbbecSDKConfig_v2.0.xml

SDK/licenses/ is left unchanged.

Example:
  $(basename "$0") \\
    /home/user/Downloads/SDK/OrbbecSDK_v2.10.2_linux_arm64.tar.gz
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

cleanup_work_dir() {
  if [[ -n "${work_dir}" && "${work_dir}" == "${PACKAGE_DIR}"/.sdk-update.* ]]; then
    rm -rf -- "${work_dir}"
  fi
}

backup_exists() {
  [[ -n "${arm64_backup_dir}" && -d "${arm64_backup_dir}" ]] ||
    [[ -n "${x64_backup_dir}" && -d "${x64_backup_dir}" ]] ||
    [[ -n "${config_backup_file}" && -e "${config_backup_file}" ]]
}

restore_backup() {
  backup_exists || return 0

  echo "Update failed; restoring the original SDK libraries and runtime configuration..." >&2
  if [[ -n "${config_backup_file}" && -e "${config_backup_file}" ]]; then
    if [[ -e "${ROS_CONFIG_FILE}" ]]; then
      mv -- "${ROS_CONFIG_FILE}" "${work_dir}/OrbbecSDKConfig.failed.xml" || return 1
    fi
    mv -- "${config_backup_file}" "${ROS_CONFIG_FILE}" || return 1
  fi

  if [[ -n "${x64_backup_dir}" && -d "${x64_backup_dir}" ]]; then
    if [[ -e "${SDK_ROOT}/x64" ]]; then
      mv -- "${SDK_ROOT}/x64" "${work_dir}/x64.failed" || return 1
    fi
    mv -- "${x64_backup_dir}" "${SDK_ROOT}/x64" || return 1
  fi

  if [[ -n "${arm64_backup_dir}" && -d "${arm64_backup_dir}" ]]; then
    if [[ -e "${SDK_ROOT}/arm64" ]]; then
      mv -- "${SDK_ROOT}/arm64" "${work_dir}/arm64.failed" || return 1
    fi
    mv -- "${arm64_backup_dir}" "${SDK_ROOT}/arm64" || return 1
  fi
}

on_exit() {
  local status=$?
  trap - EXIT

  if ((status != 0)) && backup_exists; then
    if ! restore_backup; then
      echo "Error: automatic rollback failed; recovery files remain in ${work_dir}" >&2
      exit "${status}"
    fi
  fi

  if ! backup_exists; then
    cleanup_work_dir
  fi
  exit "${status}"
}

trap on_exit EXIT
trap 'exit 130' INT TERM HUP

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --dry-run) dry_run=true; shift ;;
    --single-arch) single_arch=true; shift ;;
    --) shift; break ;;
    -*) die "unknown option: $1" ;;
    *) break ;;
  esac
done

[[ $# -eq 1 ]] || {
  usage >&2
  exit 2
}

for command_name in basename cat cmp cp dirname file find mkdir mktemp mv readlink rm wc; do
  command -v "${command_name}" >/dev/null 2>&1 || die "required command not found: ${command_name}"
done

[[ -d "${SDK_ROOT}" && ! -L "${SDK_ROOT}" ]] ||
  die "bundled SDK must be a real directory: ${SDK_ROOT}"
[[ -d "${SDK_ROOT}/arm64" && ! -L "${SDK_ROOT}/arm64" ]] ||
  die "bundled ARM64 SDK must be a real directory: ${SDK_ROOT}/arm64"
[[ -d "${SDK_ROOT}/x64" && ! -L "${SDK_ROOT}/x64" ]] ||
  die "bundled x86-64 SDK must be a real directory: ${SDK_ROOT}/x64"
[[ -d "${SDK_ROOT}/licenses" && ! -L "${SDK_ROOT}/licenses" ]] ||
  die "bundled SDK licenses must be a real directory: ${SDK_ROOT}/licenses"
[[ -f "${ROS_CONFIG_FILE}" && ! -L "${ROS_CONFIG_FILE}" ]] ||
  die "ROS SDK configuration must be a real file: ${ROS_CONFIG_FILE}"

archive_suffix=""
if [[ -d "$1" ]]; then
  :
elif [[ -f "$1" && "$1" == *.tar.gz ]]; then
  archive_suffix=".tar.gz"
  command -v tar >/dev/null 2>&1 || die "required command not found: tar"
else
  die "SDK directory or .tar.gz archive not found: $1"
fi
input_source="$(readlink -f -- "$1")"
input_source="${input_source%"${archive_suffix}"}"

case "${input_source}" in
  *_arm64) input_arch=arm64; paired_arch=x64; paired_suffix=x86_64 ;;
  *_x86_64) input_arch=x64; paired_arch=arm64; paired_suffix=arm64 ;;
  *) die "SDK name must end with _arm64 or _x86_64 (before .tar.gz): ${input_source}" ;;
esac

source_exists() {
  if [[ -n "${archive_suffix}" ]]; then
    [[ -f "$1${archive_suffix}" ]]
  else
    [[ -d "$1" ]]
  fi
}

architectures=("${input_arch}")
sources=("${input_source}")
if [[ "${single_arch}" == false ]]; then
  if [[ "${input_arch}" == arm64 ]]; then
    paired_source="${input_source%_arm64}_${paired_suffix}"
  else
    paired_source="${input_source%_x86_64}_${paired_suffix}"
  fi
  if ! source_exists "${paired_source}"; then
    candidates=()
    source_name=${input_source##*/}
    # Official names: OrbbecSDK_vVERSION_TIMESTAMP_COMMIT_linux_ARCH.
    if [[ "${source_name}" =~ ^(OrbbecSDK_v[^_]+)_[0-9]+_([^_]+)_linux_(arm64|x86_64)$ ]]; then
      prefix=${BASH_REMATCH[1]}
      commit=${BASH_REMATCH[2]}
      for candidate in "${input_source%/*}/${prefix}_"*"_${commit}_linux_${paired_suffix}${archive_suffix}"; do
        candidate=${candidate%"${archive_suffix}"}
        candidate_name=${candidate##*/}
        timestamp=${candidate_name#"${prefix}_"}
        timestamp=${timestamp%"_${commit}_linux_${paired_suffix}"}
        if [[ "${timestamp}" =~ ^[0-9]+$ ]] && source_exists "${candidate}"; then
          candidates+=("${candidate}")
        fi
      done
    fi
    paired_source=""
    if [[ ${#candidates[@]} -eq 1 ]]; then
      paired_source=${candidates[0]}
    elif [[ ${#candidates[@]} -gt 1 ]]; then
      echo "Warning: multiple paired SDK candidates; updating only ${input_arch}:" >&2
      printf '  %s\n' "${candidates[@]}" >&2
    else
      echo "Warning: paired SDK not found; updating only ${input_arch}." >&2
    fi
  fi
  if [[ -n "${paired_source}" ]]; then
    architectures+=("${paired_arch}")
    sources+=("${paired_source}")
    echo "Found paired SDK: ${paired_source}${archive_suffix}"
  fi
fi

for source_dir in "${sources[@]}"; do
  case "${source_dir}" in
    "${SDK_ROOT}" | "${SDK_ROOT}"/*) die "the source cannot be inside ${SDK_ROOT}" ;;
  esac
done

core_library_path() {
  local source_dir=$1
  local core_link="${source_dir}/lib/libOrbbecSDK.so"

  [[ -e "${core_link}" ]] || die "missing core SDK library: ${core_link}"
  readlink -f -- "${core_link}"
}

sdk_version() {
  local core_library
  core_library="$(core_library_path "$1")"
  local filename=${core_library##*/}

  [[ "${filename}" == libOrbbecSDK.so.* ]] ||
    die "cannot determine SDK version from core library: ${core_library}"
  echo "${filename#libOrbbecSDK.so.}"
}

verify_architecture() {
  local source_dir=$1
  local expected_arch=$2
  local core_library
  local description
  core_library="$(core_library_path "${source_dir}")"
  description="$(file -Lb -- "${core_library}")"

  case "${expected_arch}" in
    arm64)
      [[ "${description}" == *"ARM aarch64"* || "${description}" == *"AArch64"* ]] ||
        die "${source_dir} is not an ARM64 SDK (${description})"
      ;;
    x64)
      [[ "${description}" == *"x86-64"* ]] ||
        die "${source_dir} is not an x86-64 SDK (${description})"
      ;;
    *)
      die "internal error: unsupported architecture ${expected_arch}"
      ;;
  esac
}

verify_symlinks_within() {
  local root_dir
  local link_path
  local resolved_path
  root_dir="$(readlink -f -- "$1")"

  while IFS= read -r -d '' link_path; do
    if ! resolved_path="$(readlink -f -- "${link_path}")"; then
      die "broken symbolic link: ${link_path}"
    fi
    case "${resolved_path}" in
      "${root_dir}"/*) ;;
      *) die "symbolic link points outside ${root_dir}: ${link_path} -> ${resolved_path}" ;;
    esac
  done < <(find "${root_dir}" -type l -print0)
}

verify_source_layout() {
  local source_dir=$1
  local expected_arch=$2
  local required_path

  for required_path in \
    include/libobsensor/ObSensor.h \
    include/libobsensor/ObSensor.hpp \
    lib/OrbbecSDKConfig-release.cmake \
    lib/OrbbecSDKConfig.cmake \
    lib/OrbbecSDKVersion.cmake \
    lib/OrbbecSDKConfig.xml; do
    [[ -f "${source_dir}/${required_path}" ]] ||
      die "missing required ${expected_arch} SDK file: ${source_dir}/${required_path}"
  done

  [[ -d "${source_dir}/lib/extensions" ]] ||
    die "missing ${expected_arch} SDK extensions directory: ${source_dir}/lib/extensions"

  verify_architecture "${source_dir}" "${expected_arch}"
  verify_symlinks_within "${source_dir}/lib"
}

copy_architecture() {
  local source_dir=$1
  local target_arch=$2
  local destination_dir=$3
  local cmake_file
  local copied_core_library=false

  mkdir -p -- "${destination_dir}/include" "${destination_dir}/lib"
  cp -a -- "${source_dir}/include/libobsensor" "${destination_dir}/include/"

  while IFS= read -r -d '' library; do
    cp -a -- "${library}" "${destination_dir}/lib/"
    copied_core_library=true
  done < <(find "${source_dir}/lib" -maxdepth 1 \( -type f -o -type l \) \
    -name 'libOrbbecSDK.so*' -print0)
  [[ "${copied_core_library}" == true ]] ||
    die "no core SDK libraries were copied for ${target_arch}"

  for cmake_file in \
    OrbbecSDKConfig-release.cmake \
    OrbbecSDKConfig.cmake \
    OrbbecSDKVersion.cmake; do
    cp -a -- "${source_dir}/lib/${cmake_file}" "${destination_dir}/lib/"
  done

  cp -a -- "${source_dir}/lib/extensions" "${destination_dir}/lib/"
}

extract_sdk_archive() {
  local archive=$1
  local destination=$2
  local expected_root=${archive##*/}
  expected_root=${expected_root%.tar.gz}

  mkdir -p -- "${destination}" || return 1
  tar --extract --gzip --file="${archive}" --directory="${destination}" --no-same-owner || return 1
  # Official archives contain a directory named after the archive. Also accept
  # archives with include/ and lib/ directly at the root.
  if [[ -d "${destination}/${expected_root}" ]]; then
    printf '%s\n' "${destination}/${expected_root}"
  else
    printf '%s\n' "${destination}"
  fi
}

work_dir="$(mktemp -d -- "${PACKAGE_DIR}/.sdk-update.XXXXXX")"
for i in "${!sources[@]}"; do
  if [[ -n "${archive_suffix}" ]]; then
    sources[$i]="$(extract_sdk_archive "${sources[$i]}${archive_suffix}" \
      "${work_dir}/source-${architectures[$i]}")" || die "failed to extract SDK archive"
  fi
  verify_source_layout "${sources[$i]}" "${architectures[$i]}"
done

version="$(sdk_version "${sources[0]}")"
if [[ ${#sources[@]} -eq 2 ]]; then
  paired_version="$(sdk_version "${sources[1]}")"
  [[ "${version}" == "${paired_version}" ]] ||
    die "SDK versions do not match: ${version}, ${paired_version}"
  cmp -s -- "${sources[0]}/lib/OrbbecSDKConfig.xml" \
    "${sources[1]}/lib/OrbbecSDKConfig.xml" ||
    die "ARM64 and x86-64 OrbbecSDKConfig.xml files do not match"
else
  echo "Warning: shared runtime XML will also be updated; the other architecture's SDK is unchanged." >&2
fi

staged_sdk="${work_dir}/SDK.new"
staged_config="${work_dir}/OrbbecSDKConfig_v2.0.xml.new"
mkdir -p -- "${staged_sdk}"
for i in "${!sources[@]}"; do
  arch=${architectures[$i]}
  copy_architecture "${sources[$i]}" "${arch}" "${staged_sdk}/${arch}"
  verify_architecture "${staged_sdk}/${arch}" "${arch}"
done
cp -a -- "${sources[0]}/lib/OrbbecSDKConfig.xml" "${staged_config}"
verify_symlinks_within "${staged_sdk}"

sdk_file_count="$(find "${staged_sdk}" \( -type f -o -type l \) | wc -l)"
file_count=$((sdk_file_count + 1))
echo "Validated Orbbec SDK ${version} for: ${architectures[*]}."
echo "Staged ${file_count} ROS package files, including ${ROS_CONFIG_FILE}."

if [[ "${dry_run}" == true ]]; then
  echo "Dry run complete; SDK libraries, licenses, and runtime configuration were not changed."
  exit 0
fi

for arch in "${architectures[@]}"; do
  case "${arch}" in
    arm64) arm64_backup_dir="${work_dir}/arm64.old" ;;
    x64) x64_backup_dir="${work_dir}/x64.old" ;;
  esac
  mv -- "${SDK_ROOT}/${arch}" "${work_dir}/${arch}.old"
done
config_backup_file="${work_dir}/OrbbecSDKConfig_v2.0.xml.old"
mv -- "${ROS_CONFIG_FILE}" "${config_backup_file}"
for arch in "${architectures[@]}"; do
  mv -- "${staged_sdk}/${arch}" "${SDK_ROOT}/${arch}"
done
mv -- "${staged_config}" "${ROS_CONFIG_FILE}"

# The verified SDK and runtime configuration are now active. Mark the
# transaction as committed; the exit handler removes the temporary backups.
arm64_backup_dir=""
x64_backup_dir=""
config_backup_file=""

echo "Updated bundled SDK (${architectures[*]}) and runtime configuration to Orbbec SDK ${version}."
echo "Left ${SDK_ROOT}/licenses unchanged."
