if exist venv (
    echo Virtual environment already exists.
) else (
    echo Creating a virtual environment...
    python -m venv venv
)


call venv\Scripts\activate

if exist requirements.txt (
    echo Installing dependencies from requirements.txt...
    pip install -r requirements.txt
) else (
    echo No requirements.txt found. Skipping dependency installation.
)


echo Opening Visual Studio Code...
start cmd /k "call venv\Scripts\activate && code ."


echo Setup complete. Virtual environment created and Visual Studio Code launched.
pause