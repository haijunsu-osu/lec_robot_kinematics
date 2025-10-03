@echo off
rem build_latex2pdf.bat
rem Uses pdflatex to compile latex_source\robot_kinematics.tex and moves PDF to repo root

setlocal enabledelayedexpansion

rem Determine script directory (repo root when this .bat sits at repo root)
set "SCRIPT_DIR=%~dp0"

rem Change to the latex_source folder
pushd "%SCRIPT_DIR%latex_source" || (
  echo Failed to change directory to "%SCRIPT_DIR%latex_source"
  exit /b 1
)

rem If user passed --move-only, skip compilation and just attempt to move existing PDF
if "%1"=="--move-only" (
  goto :move_only
)

rem Check that pdflatex is available
where pdflatex >nul 2>&1
if ERRORLEVEL 1 (
  echo pdflatex was not found on PATH.
  echo Please install MiKTeX or TeX Live and ensure pdflatex is available on PATH.
  echo Suggested Windows install via winget: winget install --id=MiKTeX.MiKTeX
  echo Or install via Chocolatey: choco install miktex
  popd
  exit /b 2
)

echo Running pdflatex on robot_kinematics.tex...
pdflatex -interaction=nonstopmode "robot_kinematics.tex" %*
set "LAST_EXIT=%ERRORLEVEL%"

rem If a .bib exists or the aux references a bibliography, run bibtex
set "RUN_BIBTEX=0"
if exist references.bib set "RUN_BIBTEX=1"
if exist robot_kinematics.aux (
  findstr /I "\\bibliography" robot_kinematics.aux >nul 2>&1 && set "RUN_BIBTEX=1"
)

if "%RUN_BIBTEX%"=="1" (
  echo Running bibtex...
  bibtex "robot_kinematics"  >nul 2>&1
  if ERRORLEVEL 1 (
    echo bibtex returned an error. Check .log/.blg for details.
  ) else (
    echo bibtex completed.
  )
)

rem Run pdflatex twice more to resolve cross-references
pdflatex -interaction=nonstopmode "robot_kinematics.tex" >nul 2>&1
pdflatex -interaction=nonstopmode "robot_kinematics.tex" >nul 2>&1

set LA_TEX_EXIT=%LAST_EXIT%

if exist "robot_kinematics.pdf" (
  echo Moving generated PDF to "%SCRIPT_DIR%"
  rem Clear read-only attributes on source and destination (if present)
  attrib -R "robot_kinematics.pdf" 2>nul
  attrib -R "%SCRIPT_DIR%robot_kinematics.pdf" 2>nul
  move /Y "robot_kinematics.pdf" "%SCRIPT_DIR%robot_kinematics.pdf"
  set "MOVE_EXIT=%ERRORLEVEL%"
  if !MOVE_EXIT! NEQ 0 (
    echo Move failed with exit code !MOVE_EXIT!.
    echo Attempting PowerShell Move-Item -Force fallback...
    powershell -NoProfile -Command "Move-Item -Path 'robot_kinematics.pdf' -Destination '%SCRIPT_DIR%robot_kinematics.pdf' -Force" 2>nul
    if ERRORLEVEL 1 (
      echo PowerShell Move-Item fallback failed. Attempting copy then delete original...
    ) else (
      echo PowerShell Move-Item succeeded.
      set "MOVE_EXIT=0"
    )
    if !MOVE_EXIT! NEQ 0 (
      echo Attempting fallback: copy then delete original...
    copy /Y "robot_kinematics.pdf" "%SCRIPT_DIR%robot_kinematics.pdf"
    if ERRORLEVEL 1 (
      echo Fallback copy failed with exit code %ERRORLEVEL%.
      popd
      exit /b 1
    ) else (
      del /Q "robot_kinematics.pdf"
      if ERRORLEVEL 1 (
        echo Warning: copied file but failed to delete original.
      )
  echo Done. Output saved to "%SCRIPT_DIR%robot_kinematics.pdf" - fallback copy used
    )
  ) else (
    echo Done. Output saved to "%SCRIPT_DIR%robot_kinematics.pdf"
  )
) else (
  echo No PDF was produced in latex_source. Check pdflatex output above.
)

popd
endlocal
exit /b %LA_TEX_EXIT%

:move_only
rem Move-only label: attempt to move existing PDF without compiling
if exist "robot_kinematics.pdf" (
  echo [move-only] Moving robot_kinematics.pdf to "%SCRIPT_DIR%"
  rem Clear read-only attributes on source and destination (if present)
  attrib -R "robot_kinematics.pdf" 2>nul
  attrib -R "%SCRIPT_DIR%robot_kinematics.pdf" 2>nul
  move /Y "robot_kinematics.pdf" "%SCRIPT_DIR%robot_kinematics.pdf"
  set "MOVE_EXIT=%ERRORLEVEL%"
  if !MOVE_EXIT! NEQ 0 (
    echo [move-only] Move failed with exit code !MOVE_EXIT!. Trying PowerShell Move-Item -Force fallback.
    powershell -NoProfile -Command "Move-Item -Path 'robot_kinematics.pdf' -Destination '%SCRIPT_DIR%robot_kinematics.pdf' -Force" 2>nul
    if ERRORLEVEL 1 (
      echo [move-only] PowerShell Move-Item fallback failed. Trying fallback copy+delete.
    ) else (
      echo [move-only] PowerShell Move-Item succeeded.
      set "MOVE_EXIT=0"
    )
    if !MOVE_EXIT! NEQ 0 (
      copy /Y "robot_kinematics.pdf" "%SCRIPT_DIR%robot_kinematics.pdf"
    if ERRORLEVEL 1 (
      echo [move-only] Fallback copy failed with exit code %ERRORLEVEL%.
      popd
      endlocal
      exit /b 1
    ) else (
      del /Q "robot_kinematics.pdf"
      echo [move-only] Done. Output saved to "%SCRIPT_DIR%robot_kinematics.pdf"
      popd
      endlocal
      exit /b 0
    )
  ) else (
    echo [move-only] Done. Output saved to "%SCRIPT_DIR%robot_kinematics.pdf"
    popd
    endlocal
    exit /b 0
  )
) else (
  echo [move-only] No PDF found in latex_source to move.
  popd
  endlocal
  exit /b 1
)
