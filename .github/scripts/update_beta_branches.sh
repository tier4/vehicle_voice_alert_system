#!/usr/bin/env bash

git clone git@github.com:tier4/autoware-release-scripts.git

set -e

source "autoware-release-scripts/scripts/common/helper_functions.sh"

# Override
function get_main_repos_file_path() {
    echo "$(get_workspace_root)/.beta.repos"
}

# Check .beta.repos
if ! test -f "$(get_main_repos_file_path)"; then
    echo -e "\e[31m.beta.repos was not found.\e[m"
    exit 1
fi

# Run pre common tasks
source "autoware-release-scripts/scripts/common/pre_common_tasks.sh"

# Revert override
function get_main_repos_file_path() {
    echo "$(get_workspace_root)/vehicle_voice_alert_system.repos"
}

# Freeze vcs versions
echo -e "\e[36mFreeze vcs versions.\e[m"
freeze_vcs_versions

# Commit autoware.proj
git add "*.repos"
git diff --cached --exit-code || git commit -m "Update repos files"

echo -e "\e[36mComplete.\e[m"

rm -rf autoware-release-scripts/ src/autoware
