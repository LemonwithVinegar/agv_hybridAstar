#! /bin/zsh
find src -type f -name ".vscode" -exec rm -rf {} \;
find src -type f -name ".vs" -exec rm -rf {} \;
