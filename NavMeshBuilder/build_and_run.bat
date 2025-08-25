@echo off
echo Building Standalone Large Scale NavMesh Builder...

REM Create build directory
if not exist "navmesh_build" mkdir navmesh_build
cd navmesh_build

REM Configure with CMake (try different Visual Studio versions)
echo Trying Visual Studio 2022...
cmake .. -G "Visual Studio 17 2022" -A x64
if %errorlevel% neq 0 (
    echo Visual Studio 2022 not found, trying 2019...
    cmake .. -G "Visual Studio 16 2019" -A x64
)
if %errorlevel% neq 0 (
    echo Visual Studio 2019 not found, trying 2017...
    cmake .. -G "Visual Studio 15 2017 Win64"
)

if %errorlevel% neq 0 (
    echo ERROR: Could not configure with CMake. Please install Visual Studio with C++ tools.
    pause
    exit /b 1
)

REM Build the project
echo.
echo Building project...
cmake --build . --config Release

if %errorlevel% neq 0 (
    echo ERROR: Build failed.
    pause
    exit /b 1
)

REM Run the navmesh builder
echo.
echo Building navmesh for NAVTest137Wipv4v3.obj...
echo This may take a while for a 2GB mesh...
echo.

bin\Release\NavMeshBuilder.exe ..\..\..\build\RecastDemo\Release\Meshes\NAVTest137Wipv4v3.obj navmesh_output.bin

if %errorlevel% equ 0 (
    echo.
    echo Success! Testing the generated navmesh...
    echo.
    bin\Release\NavMeshLoader.exe navmesh_output.bin
    echo.
    echo Build completed successfully!
) else (
    echo.
    echo Build failed with error code %errorlevel%
    echo Check that the OBJ file path is correct.
)

echo.
echo Press any key to continue...
pause > nul