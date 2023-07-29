from conan.tools.files import copy, rmdir
import os


def deploy(graph, output_folder, **kwargs):
    import os
    import shutil

    output_folder = os.path.join(output_folder, "direct_deploy")
    conanfile = graph.root.conanfile
    conanfile.output.info(f"Conan relocatable pkg deployer to {output_folder}")
    # If the argument is --requires, the current conanfile is a virtual one with 1 single
    # dependency, the "reference" package. If the argument is a local path, then all direct
    # dependencies
    rmdir(graph.root.conanfile, output_folder)
    for key, dep in conanfile.dependencies.host.items():
        if dep.package_folder is None:
            print("Skip: {} - no package folder".format(key))
            continue
        print("Deploy pkg: {} from: {}".format(str(dep), dep.package_folder))
        shutil.copytree(dep.package_folder, output_folder, symlinks=True, dirs_exist_ok=True)
        dep.set_deploy_folder(output_folder)