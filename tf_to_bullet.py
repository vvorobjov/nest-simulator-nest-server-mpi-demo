from nrp_core import *
from nrp_core.data.nrp_json import *
import OUnoise2018 as ou



@EngineDataPack(keyword='positions', id=DataPackIdentifier('positions', "bullet_simulator"))
#@EngineDataPack(keyword='spikes', id=DataPackIdentifier('spikes', 'nest'))
@TransceiverFunction("bullet_simulator")
def to_bullet(positions):
    rec_cmd = JsonDataPack("control_cmd", "bullet_simulator")

    if not 'noise' in globals():
        global noise
        print("In TF to")
        print(type(positions.data["hand"]))
        #print(positions.data[0].size())
        noise = ou.OUNoise(len(positions.data["hand"]),1)

    #car_pos = positions.data["body"]
    
    noize = noise.sample()

    
    
    rec_cmd.data["act_list"] = noize

    return [rec_cmd]