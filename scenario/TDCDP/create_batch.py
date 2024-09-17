import os

folders = [x for x in os.listdir('scenario/TDCDP') if '.py' not in x \
                                                    and '.scn' not in x]

batchscn = open("scenario/TDCDP/batchtdcdp.scn", "w")

for folder in folders:
    files = os.listdir(f'scenario/TDCDP/{folder}')
    for file in files:    
        if 'Seattle' in file:
            continue
        batchscn.write(f'00:00:00.00>SCEN {folder}_{file}\n')
        batchscn.write(f'00:00:00.00>PCALL {folder}/{file}\n')
        batchscn.write('00:00:00.00>FF\n\n')
