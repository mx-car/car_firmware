# car_firmware
teensy car firmware
## git
The project includes sub repositories, therefore you have to use the following command to clone everything

```
git clone --recurse-submodules git@github.com:mx-car/car_firmware.git
```
The command above will clone a specific version of the sub-repository. 
### update sub-repository
If you like to get the newest sub-repository you have to checkout the master branch in each sub-repository.
```
git checkout master
```
## Compilation
### issues
#### M4l_math   
* Compilation might fail due to a missing math library since we are using a newer compiler than the stardart issue. In this case simply copy the library from an older gccarmnoneaebi:  

  ```
  cp ~/.platformio/packages/toolchain-gccarmnoneeabi@1.50401.190816/arm-none-eabi/lib/libarm_cortexM4l_math.a ~/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/lib
  ```

  For manual copying you can look for the missing library under ``~/.platformio/packages``.  Try `find ~/.platformio -iname libarm_cortexM4l_math.a` to locate the missing library if the command above doesn't work directly.
