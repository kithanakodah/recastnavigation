@echo off
echo Building NavMesh Checker and Analysis Tools...

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

echo.
echo ======================================
echo Build completed successfully!
echo ======================================
echo.
echo Available tools:
echo   NavMeshChecker.exe  - Analyze OBJ meshes for NavMesh requirements
echo   NavMeshLoader.exe   - Load and test existing NavMesh files
echo.
echo Usage examples:
echo   bin\Release\NavMeshChecker.exe "C:\path\to\your\mesh.obj"
echo   bin\Release\NavMeshChecker.exe "C:\path\to\your\mesh.obj" --config
echo   bin\Release\NavMeshLoader.exe "C:\path\to\navmesh.bin" test
echo.
echo To use the tools, specify the full path to your files.
echo.
echo Press any key to continue...
pause > nul