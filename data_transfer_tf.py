from nrp_core import *
from nrp_core.data.nrp_protobuf import DumpStringDataPack

import numpy as np


@EngineDataPack(keyword='brainstem_n', id=DataPackIdentifier('spikedetector_brain_stem_neg', 'nest'))
@EngineDataPack(keyword='brainstem_p', id=DataPackIdentifier('spikedetector_brain_stem_pos', 'nest'))
@TransceiverFunction("datatransfer_engine")
def data_transfer(brainstem_n, brainstem_p):

    print("Got brainstem_n population of size", len(brainstem_n.data))
    print("Got brainstem_p population of size", len(brainstem_p.data))

    # Streaming formatted string
    string_datapack = DumpStringDataPack("test_datapack1", "datatransfer_engine")
    string_datapack.data.string_stream = f"{len(brainstem_n.data):.5f}"

    string_datapack2 = DumpStringDataPack("test_datapack2", "datatransfer_engine")
    string_datapack.data.string_stream = f"{len(brainstem_p.data):.5f}"


    return [string_datapack, string_datapack2]
