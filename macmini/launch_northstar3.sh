export GENICAM_CACHE_V3_1=/tmp/genicam_cache_3;
cd ~/northstar
source ./venv/bin/activate

while [ True ]; do
  /Users/frc6328/northstar/reenumerate/reenumerate -v -l 0x00100000
  sudo nice -n -20 sudo -u frc6328 python3 __init__.py --config config_3.json --calibration calibration_3.json
  sleep 1
done
