# Load normal terminal settings, then activate this project's ESP-IDF setup.
if [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc"
fi

# VS Code may still carry the old Xilinx environment until it is restarted.
unset LD_LIBRARY_PATH
source /home/eros-nucifora/.espressif/tools/activate_idf_v6.1.sh
export IDF_TARGET=esp32
