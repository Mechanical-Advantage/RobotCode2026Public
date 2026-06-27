export GENICAM_CACHE_V3_1=/tmp/genicam_cache_0;
cd ~/northstar
source ./venv/bin/activate

while [ True ]; do
  uhubctl -a cycle -l 2-2 -p 2 -d 1 -N
  sleep 2
  /Users/frc6328/northstar/reenumerate/reenumerate -v -l 0x02120000
  sudo nice -n -20 sudo -u frc6328 python3 __init__.py --config config_0.json --calibration calibration_0.json
  sleep 1
done
