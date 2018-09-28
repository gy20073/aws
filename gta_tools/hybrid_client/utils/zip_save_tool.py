import os
import zipfile
import shutil
from tqdm import tqdm


def zipdir(path2zip, output_name):
    file_list = os.listdir(path2zip)
    with zipfile.ZipFile(output_name, 'w') as myzip:
        for f in tqdm(file_list):
            myzip.write(os.path.join(path2zip, f), arcname=f)
        myzip.close()


def zip_to_file(path2zip, output_name):
    file_list = os.listdir(path2zip)
    with zipfile.ZipFile(output_name, 'w') as myzip:
        for f in tqdm(file_list):
            myzip.write(os.path.join(path2zip, f), arcname=f)
        myzip.close()


def zip_and_remove(path2zip, output_name):
    zip_to_file(path2zip, output_name)
    shutil.rmtree(path2zip)


def check_md5(file_name):
    import hashlib
    hash_md5 = hashlib.md5()
    with open(file_name, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()
