#!/bin/bash
# This is a script to update all local branches to match remote branches.
# To run it, go to the repo you want to update and execute: "~/Sandbox/sync_branches.sh"
# Get all local branches
branches=$(git branch | sed 's/^[* ] //')

for branch in $branches; do
    echo "----------------------------"
    echo "Changing to branch: $branch"
    git checkout $branch

    echo "Pulling  $branch from remote"
    git pull origin $branch
done

echo "----------------------------"
echo "All local branches has been updated."
