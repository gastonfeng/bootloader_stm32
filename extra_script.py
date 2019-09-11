import os

Import("env", "projenv")

# access to global construction environment
# print env

# access to project construction environment
# print projenv

# Dump construction environments (for debug purpose)
# print env.Dump()
# print projenv.Dump()

BUILD_NUMBER = os.environ.get('BUILD_NUMBER') or "0"
projenv.Append(CPPDEFINES=[('BUILD_NUMBER', BUILD_NUMBER)])


# env.Append(ldscript='stm32f103ve.ld')
def before_upload(source, target, env):
    print
    "before_upload"
    # do some actions


def after_upload(source, target, env):
    print
    "after_upload"
    # do some actions


env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
