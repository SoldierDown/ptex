import os
root_path = './ptx_files/'
ptx_file = 'mask_black_64_write.ptx'
'''
  -v Show ptex software version
  -m Dump meta data
  -f Dump face info
  -d Dump data
  -D Dump data for all mipmap levels
  -t Dump tiling info
  -i Dump internal info
  -c Check validity of adjacency data
'''
output_root_path = ptx_file.replace('.ptx', '_info')
if not os.path.exists(output_root_path):
    os.mkdir(output_root_path)

os.system('./install/bin/ptxinfo -m {}{} > ./{}/meta_data.txt'.format(root_path, ptx_file, output_root_path))
os.system('./install/bin/ptxinfo -f {}{} > ./{}/face_info.txt'.format(root_path, ptx_file, output_root_path))
os.system('./install/bin/ptxinfo -t {}{} > ./{}/tiling.txt'.format(root_path, ptx_file, output_root_path))
os.system('./install/bin/ptxinfo -i {}{} > ./{}/internal.txt'.format(root_path, ptx_file, output_root_path))
os.system('./install/bin/ptxinfo -c {}{} > ./{}/adj_validity.txt'.format(root_path, ptx_file, output_root_path))