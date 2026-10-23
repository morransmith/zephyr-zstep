#!/bin/sh
# SPDX-License-Identifier: Apache-2.0

set -eu

current=$(svu current)
next=$(svu next)

if [ "$next" = "$current" ]; then
	echo "No version change since $current"
	exit 0
fi

# Gitea creates the tag together with the release.
wget -q -O - \
	--header "Authorization: token $GITEA_TOKEN" \
	--header "Content-Type: application/json" \
	--post-data "{\"tag_name\":\"$next\",\"target_commitish\":\"$CI_COMMIT_SHA\",\"name\":\"$next\"}" \
	"$CI_FORGE_URL/api/v1/repos/$CI_REPO/releases"
