#
# Created by pengguoqi on 20-03-24.
#

import sys
import os
import zipfile

def zipDir(dirpath,outFullName):
    zip = zipfile.ZipFile(outFullName,"w",zipfile.ZIP_DEFLATED)
    for path,dirnames,filenames in os.walk(dirpath):
        fpath = path.replace(dirpath,'')
        for filename in filenames:
            zip.write(os.path.join(path,filename),os.path.join(fpath,filename))
    zip.close()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        files = path + "/gmm/"
        zip_file = path + '/gmm.zip'
        zipDir(files,zip_file)