#!/usr/bin/env python3
"""
@author: Benjamin Perseghetti
@email: bperseghetti@rudislabs.com
"""
import jinja2
import os
import numpy as np
import argparse
import xmltodict

configPath = os.path.realpath(__file__).replace("jinja_gen.py","")
cerebriPath = os.path.relpath(os.path.join(configPath, ".."))

fileGenList = [
    "drivers/sim_gz/src/sim_gz_driver.cc.jinja",
    "src/controller.c.jinja"
]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--hcdf', default="x500.hcdf", help="hardware configuration description file name in config folder")
    parser.add_argument('--clean', help="Clean Build")

    args = parser.parse_args()

    fileHCDF = os.path.realpath(os.path.join(configPath, args.hcdf))
    with open(fileHCDF) as fd:
        config = xmltodict.parse(fd.read(), process_namespaces=True)

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(cerebriPath))

    for file in fileGenList:

        filePath = os.path.relpath(os.path.join(cerebriPath, file))
        if os.path.isfile(filePath):
            template = env.get_template(filePath)

            result = template.render(config)

            filenameOut = filePath.replace('.jinja','')
            
            if (not os.path.isfile(filenameOut)) or (args.clean is not None):
                with open(filenameOut, 'w') as f_out:
                    print(f"{filePath} -> {filenameOut}")
                    f_out.write(result)
            else:
                tmpFilenameOut = f"{filenameOut}.tmp"
                print(f"{filenameOut} already exists writing to: {tmpFilenameOut}")
                with open(tmpFilenameOut, 'w') as f_out:
                    print(f"{filePath} -> {tmpFilenameOut}")
                    f_out.write(result)
        else:
            print(f"{filePath} does not exist!!!")