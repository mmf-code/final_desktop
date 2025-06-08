# Cursor IDE Setup for Multi-Agent Formation Control Project

This document provides setup instructions for using Cursor IDE with this ROS project.

## Issues Resolved

The following issues have been addressed for Cursor compatibility:

1. **YAML Language Support**: Added proper YAML validation and IntelliSense
2. **C++ IntelliSense**: Enhanced clangd configuration for better code completion
3. **ROS Integration**: Added ROS-specific file associations and settings
4. **Project Structure**: Created workspace configuration for multi-package setup

## Required Extensions

Install these extensions in Cursor for optimal development experience:

### Essential Extensions
- **C/C++** (`ms-vscode.cpptools`) - C++ IntelliSense and debugging
- **CMake Tools** (`ms-vscode.cmake-tools`) - CMake integration
- **YAML** (`redhat.vscode-yaml`) - YAML language support with validation
- **ROS** (`ms-iot.vscode-ros`) - ROS development tools
- **clangd** (`llvm-vs-code-extensions.vscode-clangd`) - Advanced C++ language server

### Recommended Extensions
- **Python** (`ms-python.python`) - For Python scripts
- **XML** (`redhat.vscode-xml`) - For launch files and URDF
- **GitLens** (`eamodio.gitlens`) - Enhanced Git integration

## Setup Steps

1. **Open the workspace file**:
   ```
   File → Open Workspace from File → multi-agent-formation-control.code-workspace
   ```

2. **Install recommended extensions**:
   - Cursor will prompt you to install recommended extensions
   - Click "Install All" when prompted

3. **Configure CMake**:
   - Press `Ctrl+Shift+P` and run "CMake: Configure"
   - Select the appropriate compiler (MSVC 2022)

4. **Build the project**:
   - Press `Ctrl+Shift+P` and run "CMake: Build"
   - Or use `F7` shortcut

## Configuration Files

The following configuration files have been created/updated:

- `.vscode/settings.json` - Enhanced with YAML support and ROS settings
- `.vscode/extensions.json` - Recommended extensions list
- `.clangd` - Improved C++ language server configuration
- `schemas/ros-param-schema.json` - YAML schema for parameter validation
- `multi-agent-formation-control.code-workspace` - Workspace configuration

## YAML Support

YAML files now have:
- Syntax highlighting
- Validation against ROS parameter schemas
- Auto-completion for common ROS parameters
- Proper formatting on save

## Troubleshooting

### C++ IntelliSense Issues
1. Ensure compile_commands.json is generated: `CMake: Configure`
2. Restart the C++ language server: `Ctrl+Shift+P` → "C/C++: Restart IntelliSense"
3. Check that vcpkg path is correct in settings.json
4. If yaml-cpp headers are not found:
   ```powershell
   cd C:\vcpkg
   .\vcpkg.exe install yaml-cpp:x64-windows
   ```
5. Restart clangd: `Ctrl+Shift+P` → "clangd: Restart language server"

### YAML Issues
1. Verify YAML extension is installed and enabled
2. Check that schema files are accessible
3. Restart Cursor if validation doesn't work

### Build Issues
1. Ensure vcpkg is properly installed at `C:/vcpkg/`
2. Check that Visual Studio 2022 is installed
3. Verify CMake toolchain file path in settings
4. Regenerate build files:
   ```powershell
   cmake --build build --config Release
   ```

### Missing Dependencies
If you get "file not found" errors for includes:
1. Install missing packages via vcpkg
2. Rebuild the project to regenerate compile_commands.json
3. Restart language servers in Cursor

## Performance Optimization

The configuration excludes the following directories from indexing:
- `build/`
- `install/`
- `log/`
- `vcpkg_installed/`
- `.venv/`

This improves Cursor's performance and reduces memory usage.

## Additional Notes

- The project uses C++17 standard
- clangd is configured with enhanced diagnostics
- Format on save is enabled for consistent code style
- Search excludes build artifacts for faster searches 