# -*- coding:utf-8 -*-
#*************************************************************************
#	> File Name: package_script.py
#	> Author: ThierryCao
#	> Mail: sycao@listenai.com
#	> Created Time: Thu 28 May 2020 12:54:08 PM CST
# ************************************************************************
import os
import platform
#pyinstaller -F .\iflyos_dumpAudio.py --additional-hooks-dir=.

def get_current_file_path():
    return os.path.dirname(os.path.abspath(__file__) )

def join_path(*args, **argv):
    if args == 2:
        return os.path.join(argv[0], argv[1])


def get_platform_system():
    return platform.system()
def isPlatformWin():
    return True if get_platform_system() == 'Windows' else False

def get_python_root_dir():
    import sys
    python_root_dir = ''
    if not isPlatformWin():
        python_root_dir = [i for i in sys.path if 'site-packages' in i][0]
    return python_root_dir

def build_dist():
    from PyInstaller.__main__ import run
    if isPlatformWin():
        
        opts = ['-F', 
                # '--paths=H:\\tools\\dev\\Anaconda3\\envs\\py37',             
                # '--paths=H:\\tools\\dev\\Anaconda3\\envs\\py37\\Lib\\site-packages',
                '--paths={}'.format(get_python_root_dir()),
                # '--add-data={};shell'.format(os.path.join(get_current_file_path(), 'shell')),
                # '--i={}'.format(os.path.join(get_current_file_path(), '../ui/favorite/_ico.ico')),
                # '--upx-dir=D:\\workshop\\home\\workshop\\abc\\script\\core\\aux\\utils\\',
                '--additional-hooks-dir=.',
                '--name=comtool',
                '--distpath={}'.format(os.path.join(get_current_file_path(), '../dist')),
                '--specpath={}'.format(os.path.join(get_current_file_path(), '../dist/spec')),
                # '--add-data=../../src/demo/spec/res;res',
                # '--i=../../src/demo/spec/favorite/_ico.ico',
                # '--distpath=../dist',
                # '--specpath=../dist/spec',
                '--clean',
                'comtool.py']

        run(opts)
    else:
        opts = ['-F', 
                # '--paths=/Users/abc/workshop/abc/project/utils/anaconda3/envs/py37',             
                # '--paths=/Users/abc/workshop/abc/project/utils/anaconda3/envs/py37/lib/python3.7/site-packages',
                '--paths={}'.format(get_python_root_dir()),
                # '--add-data={}:shell'.format(os.path.join(get_current_file_path(), 'shell')),
                # '--i={}'.format(os.path.join(get_current_file_path(), '../ui/favorite/_ico.ico')),
                #'--upx-dir=D:\\workshop\\home\\workshop\\abc\\script\\core\\aux\\utils\\',
                '--additional-hooks-dir=.',
                '--name=comtool',
                '--distpath={}'.format(os.path.join(get_current_file_path(), '../dist')),
                # '--specpath={}'.format(os.path.join(get_current_file_path(), '../dist/spec')),
                # '--add-data=../../src/demo/spec/res;res',
                # '--i=../../src/demo/spec/favorite/_ico.ico',
                # '--distpath=../dist',
                # '--specpath=../dist/spec',
                '--clean',
                'comtool.py']

        run(opts)
        '''
        opts = ['-F', '--paths=/home/abc/workshop/abc/tools/anaconda3/envs/py37-tools/bin/python',
                '--clean',
                'comtool.py'
                ]
        run(opts)
        '''
def main():
    build_dist()
if __name__ == '__main__':
   main()
