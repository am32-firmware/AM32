/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
brew install make
make arm_sdk_install
cp .vscode/settings.json.macos .vscode/settings.json