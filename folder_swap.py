import os
import shutil

ls_official = os.listdir("./ROAR_Sim_Storage_OFFICIAL")

print(ls_official)

if len(ls_official) == 0:
    mode = "official"

else:
    mode = "ours"

print("previous mode: ", mode)

if mode == "official":
    shutil.move("ROAR_Sim", "ROAR_Sim_Storage_OFFICIAL/ROAR_Sim")
    shutil.move("ROAR_Sim_Storage_OURS/ROAR_Sim", "ROAR_Sim")
    mode = "ours"

else:
    shutil.move("ROAR_Sim", "ROAR_Sim_Storage_OURS/ROAR_Sim")
    shutil.move("ROAR_Sim_Storage_OFFICIAL/ROAR_Sim", "ROAR_Sim")
    mode = "official"

print("current mode: ", mode)