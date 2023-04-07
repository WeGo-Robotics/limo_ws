#!/bin/bash

# Get the current directory path
current_path=$(pwd)

# Change directory to the parent directory of the folders containing the Python files
cd "$current_path"

sudo apt install libgoogle-glog-dev -y

# Build work space
catkin_make

sudo find . -name "*.py" -type f -exec chmod 777 {} \;

# Change permissions of all Python files in subdirectories
SHELL_TYPE="$(basename "$SHELL")"
echo "--------------------"  
echo "Shell Type : $SHELL_TYPE"
if [ "$SHELL_TYPE" == "bash" ]; then
  # Bash shell
  if ! grep -q "source $current_path/devel/setup.bash" ~/.zshrc; then
    echo "source $current_path/devel/setup.bash" >> ~/.bashrc
    echo "Added 'source $current_path/devel/setup.bash' to ~/.bashrc"
    else
    echo "'source $current_path/devel/setup.bash' is already created in the bashrc file."
  fi
elif [ "$SHELL_TYPE" == "zsh" ]; then
  # Zsh shell
	if ! grep -q "source $current_path/devel/setup.zsh" ~/.zshrc; then
    echo "source $current_path/devel/setup.zsh" >> ~/.zshrc
    echo "Added 'source $current_path/devel/setup.zsh' to ~/.zshrc"
    else
    echo "'source $current_path/devel/setup.zsh' is already created in the zshrc file."
  fi
else
  echo "Unsupported shell type: $SHELL_TYPE"
  exit 1
fi

# Setup Completed
echo "Setup completed."
echo "--------------------"