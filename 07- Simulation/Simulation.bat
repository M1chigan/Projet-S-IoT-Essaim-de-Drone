@echo off
echo Lancement de WSL et d'ArduPilot...

:: Lance le script Linux dans une nouvelle fenêtre
start "ArduPilot WSL" wsl bash -ic "~/lancer.sh"

:: Attend 5 secondes puis lance Mission Planner (décommentez si besoin)
timeout /t 5 /nobreak > nul
:: start "" "C:\Program Files (x86)\Mission Planner\MissionPlanner.exe"