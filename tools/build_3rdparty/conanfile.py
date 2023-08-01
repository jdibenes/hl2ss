from conan import ConanFile


class Build3rdpartyConan(ConanFile):

	default_options = {
	 "hl2comm_idl/*:shared": False,
	 "fast-cdr/*:shared": False,
	 "zdepth/*:shared": False,
	 "spdlog/*:shared": False,
	 "rapidjson/*:shared": False,
	}

	settings = "os",
	generators = "VirtualRunEnv",

	def requirements(self):
		# self.requires("zenoh-cpp/0.10.0-dev@camposs/stable")
		self.requires("hl2comm_idl/0.0.1@artekmed/stable")
		self.requires("fast-cdr/1.0.27")
		self.requires("zdepth/0.2@camposs/stable")
		self.requires("uriparser/0.9.3")
		self.requires("spdlog/1.11.0")
		self.requires("rapidjson/cci.20220822")

