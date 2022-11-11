
# Webpyux Closed Beta Test

## how to run:
- on Linux
```bash
git clone --recurse-submodules https://github.com/AlyShmahell/webpyux-cbt
cd webpyux-cbt
python -m venv .env
source .env/bin/activate
pip install -r requirements.txt
python server.py
```
- on Windows PowerShell
```bash
git clone --recurse-submodules https://github.com/AlyShmahell/webpyux-cbt
cd webpyux-cbt
python -m venv .env
.env/Scripts/Activate.ps1
pip install -r requirements.txt
python server.py
```
## Results
- startup process
![startup process](results/startup.gif)
- spinner
![spinner](results/spinner.png)
- main screen (bypasses the login page at the moment for some reason )
![main screen](results/screen-1.png)
- error message inside the dom
![error message](results/error-1.png)
- error message inside the console
![error message](results/error-2.png)