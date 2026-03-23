sleep 5
cd ~/Desktop/NJIT_Vision/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./sentry.sh; ./camera_setting.sh"