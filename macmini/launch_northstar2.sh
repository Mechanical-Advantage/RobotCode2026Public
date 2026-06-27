export GENICAM_CACHE_V3_1=/tmp/tmp2;
cd ~/northstar
source ./venv/bin/activate

while [ True ]; do
  /Users/frc6328/northstar/reenumerate/reenumerate -v -l 0x01100000
  sudo nice -20 sudo -u frc6328 python3 __init__.py --config config_2.json --calibration calibration_2.yml
  sleep 1
done
