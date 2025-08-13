# Installation of RT Kernel for FCI

1. First, it is necessary to create a folder to run the following commands (I named it `rt_kernel`)

2. Install the necessary dependencies
    ```bash
    sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
    ```

3. Determine the kernel version you will be using. Find your current version with the command
    ```bash
    uname -r
    ```

4. Download the source files
    ```bash
    curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
    curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
    curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
    curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign
    ```
    Then decompress those files
    ```bash
    xz -d *.xz
    ```

5. Next, verify the installation
    > Run these commands one at a time, not together
    ```bash
    gpg2 --verify linux-*.tar.sign
    gpg2 --verify patch-*.patch.sign
    ```

    There may be an error thrown, with the last time saying `gpg: Can't check signiture: No public key`. If this is the case, you must retrieve the key with the following command

    ```bash
    gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys <RSA_keyID_from_err_msg>
    ```