#!/bin/bash

# Function to check if a directory exists
check_dir() {
  if [[ ! -d "$1" ]]; then
    echo "❌ Directory $1 does not exist."
    exit 1
  fi
}

# Function to update CMakeLists.txt
update_cmake() {
  local project_dir="$1"
  local cmf="$project_dir/CMakeLists.txt"
  local backup="$cmf.bak"

  echo "⮕ Backing up CMakeLists.txt to $backup"
  cp "$cmf" "$backup"

  # Modify CMakeLists.txt to reflect changes in folder structure
  echo "⮕ Updating CMakeLists.txt for new source structure"
  sed -i -e 's|/source|/source/src|g' "$cmf"  # Adjust source path for .c files
  sed -i -e 's|/source|/source/inc|g' "$cmf"  # Adjust include path for .h files

  echo "⮕ CMakeLists.txt updated successfully"
}

# Function to refactor project by moving files
refactor_project() {
  local project_dir="$1"

  echo "⮕ Starting refactor for $project_dir"
  check_dir "$project_dir/source"
  check_dir "$project_dir/source/src"
  check_dir "$project_dir/source/inc"

  # Create src and inc directories
  mkdir -p "$project_dir/source/src"
  mkdir -p "$project_dir/source/inc"

  # Move .c files to src
  mv "$project_dir/source"/*.c "$project_dir/source/src/"

  # Move .h files to inc
  mv "$project_dir/source"/*.h "$project_dir/source/inc/"

  echo "⮕ Refactor complete"
}

# Function to rollback refactor
rollback_refactor() {
  local project_dir="$1"

  echo "⮕ Starting rollback for $project_dir"
  check_dir "$project_dir/source/src"
  check_dir "$project_dir/source/inc"

  # Move .c files back to source
  mv "$project_dir/source/src"/*.c "$project_dir/source/"

  # Move .h files back to source
  mv "$project_dir/source/inc"/*.h "$project_dir/source/"

  echo "⮕ Rollback complete"
}

# Main script
echo "Welcome to the project refactor tool!"

# Ask user for input: whether to refactor or rollback
echo "Please choose an option:"
echo "1. Refactor"
echo "2. Rollback"
read -p "Enter your choice [1/2]: " choice

# Ask for the project directory
read -p "Enter the project directory (relative path): " project_dir

# Check that project directory exists
check_dir "$project_dir"

# Based on the user's choice, either refactor or rollback
if [[ "$choice" -eq 1 ]]; then
  echo "⮕ Proceeding with refactor"
  refactor_project "$project_dir"
  update_cmake "$project_dir"  # Update CMakeLists.txt for the refactored structure
  echo "🎉 Refactor complete. Please regenerate build files and build your project manually if needed."
elif [[ "$choice" -eq 2 ]]; then
  echo "⮕ Proceeding with rollback"
  rollback_refactor "$project_dir"
  update_cmake "$project_dir"  # Update CMakeLists.txt for the original structure
  echo "🎉 Rollback complete. Please regenerate build files and build your project manually if needed."
else
  echo "❌ Invalid option. Exiting."
  exit 1
fi

# Regenerate CMake build files
echo "⮕ Regenerating CMake build system…"
mkdir -p "$project_dir/out/build/debug"
cd "$project_dir/out/build/debug"
cmake ../.. -DCMAKE_BUILD_TYPE=debug -G Ninja || {
  echo "❌ CMake configure failed!"
  exit 1
}
echo "✓ CMake configure OK"

# Build the project
echo "⮕ Building the project…"
cmake --build . --target all || {
  echo "❌ Build failed!"
  exit 1
}
echo "🎉 Build succeeded!"

