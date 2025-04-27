#!/bin/bash

# Function to handle errors
error_exit() {
    echo "$1" 1>&2
    exit 1
}

# Function to get project folder name
get_project_folder() {
    read -p "Enter the name of the project folder to refactor: " PROJECT_FOLDER
    if [ ! -d "$PROJECT_FOLDER" ]; then
        error_exit "Error: Project folder '$PROJECT_FOLDER' not found!"
    fi
}

# Function to check and move the main file
move_main_file() {
    MAIN_FILE="$PROJECT_FOLDER/$PROJECT_FOLDER.c"
    if [ -f "$MAIN_FILE" ]; then
        echo "Found main file: $MAIN_FILE"
        mv "$MAIN_FILE" "$PROJECT_FOLDER/source/src/main.c" || error_exit "Failed to move main file"
    else
        error_exit "Error: Main C file '$MAIN_FILE' not found!"
    fi
}

# Function to handle file movement and directory creation
move_main_file() {
    # Check if the main file exists in the project folder
    if [ -f "$PROJECT_FOLDER/$PROJECT_FOLDER.c" ]; then
        # Check if main.c already exists in the target location
        if [ -f "$PROJECT_FOLDER/source/src/main.c" ]; then
            echo "Warning: 'main.c' already exists in the destination folder."
            read -p "Do you want to overwrite it? (y/n): " overwrite
            if [ "$overwrite" == "y" ]; then
                mv "$PROJECT_FOLDER/$PROJECT_FOLDER.c" "$PROJECT_FOLDER/source/src/main.c"
                echo "Main file overwritten."
            else
                echo "Main file not moved. Please resolve the conflict manually."
            fi
        else
            mv "$PROJECT_FOLDER/$PROJECT_FOLDER.c" "$PROJECT_FOLDER/source/src/main.c"
            echo "Main file moved successfully."
        fi
    else
        echo "Error: Main C file '$PROJECT_FOLDER/$PROJECT_FOLDER.c' not found!"
        exit 1
    fi
}

# Function to update the CMakeLists.txt to include the new structure
update_cmake_lists() {
    CMAKE_FILE="$PROJECT_FOLDER/CMakeLists.txt"

    if [ ! -f "$CMAKE_FILE" ]; then
        error_exit "Error: CMakeLists.txt file not found in the project folder."
    fi

    # Add include directories for source/inc
    sed -i "/project(/a \\ninclude_directories(\${PROJECT_SOURCE_DIR}/source/inc)" "$CMAKE_FILE"
    
    # Add source files for source/src
    sed -i "/project(/a \\nfile(GLOB SOURCES \${PROJECT_SOURCE_DIR}/source/src/*.c)" "$CMAKE_FILE"

    # Add new target for executable
    sed -i "/project(/a \\nadd_executable(\${PROJECT_NAME} \${SOURCES})" "$CMAKE_FILE"

    echo "CMakeLists.txt has been updated successfully."
}

# Function to rollback the changes
rollback_changes() {
    echo "Rolling back the refactoring..."
    mv "$PROJECT_FOLDER/source/src/main.c" "$PROJECT_FOLDER/$PROJECT_FOLDER.c"
    mv "$PROJECT_FOLDER/source/src/"*.c "$PROJECT_FOLDER/"
    mv "$PROJECT_FOLDER/source/inc/"*.h "$PROJECT_FOLDER/"
    rmdir "$PROJECT_FOLDER/source/src" "$PROJECT_FOLDER/source/inc"
    echo "Rollback completed."
}

# Main refactoring function
refactor_project() {
    get_project_folder

    # Move main file and other files to the new structure
    move_main_file
    move_files

    # Update CMakeLists.txt
    update_cmake_lists

    # Final message
    echo "Refactoring completed for '$PROJECT_FOLDER'."
}

# Prompt for rollback or refactor
echo "Do you want to (r)efactor or (b)ackup/rollback? (r/b)"
read ACTION
case $ACTION in
    r|R)
        refactor_project
        ;;
    b|B)
        echo "Please provide the project folder name to rollback:"
        get_project_folder
        rollback_changes
        ;;
    *)
        echo "Invalid option. Exiting."
        exit 1
        ;;
esac

# After refactoring, prepare build folder and run cmake
echo "Preparing build folder..."
mkdir -p "$PROJECT_FOLDER/build"

# Run cmake on the project
echo "Running cmake..."
cd "$PROJECT_FOLDER/build" || exit
cmake .. || exit

# Option to run make
echo "Do you want to run 'make' now? (y/n)"
read RUN_MAKE
if [ "$RUN_MAKE" == "y" ]; then
    make || exit
    echo "Build completed."
else
    echo "Build skipped."
fi

