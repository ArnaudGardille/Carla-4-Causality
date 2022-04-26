
cd ~/CARLA_0.9.6
./CarlaUE4.sh -windowed -carla-port=2000

cd ~/gym-carla
conda activate gym_carla_env

python test.py
python manual_agent.py
