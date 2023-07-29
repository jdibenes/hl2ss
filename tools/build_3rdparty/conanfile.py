from conan import ConanFile


class Build3rdpartyConan(ConanFile):

	default_options = {
	 "hl2comm_idl/*:shared": False,
	 "fast-cdr/*:shared": False,
	 "zdepth/*:shared": False,
	}

	settings = "os",
	generators = "VirtualRunEnv",

	def requirements(self):
		self.requires("hl2comm_idl/0.0.1@artekmed/stable")
		self.requires("fast-cdr/1.0.27")
		self.requires("zdepth/0.2@camposs/stable")
