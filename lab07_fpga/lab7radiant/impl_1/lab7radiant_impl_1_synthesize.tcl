if {[catch {

# define run engine funtion
source [file join {C:/lscc/radiant/2024.1} scripts tcl flow run_engine.tcl]
# define global variables
global para
set para(gui_mode) "1"
set para(prj_dir) "C:/Users/vparizot/E155/lab7mvp/lab07_fpga/lab7radiant"
# synthesize IPs
# synthesize VMs
# synthesize top design
file delete -force -- lab7radiant_impl_1.vm lab7radiant_impl_1.ldc
::radiant::runengine::run_engine_newmsg synthesis -f "lab7radiant_impl_1_lattice.synproj" -logfile "lab7radiant_impl_1_lattice.srp"
::radiant::runengine::run_postsyn [list -a iCE40UP -p iCE40UP5K -t SG48 -sp High-Performance_1.2V -oc Industrial -top -w -o lab7radiant_impl_1_syn.udb lab7radiant_impl_1.vm] [list C:/Users/vparizot/E155/lab7mvp/lab07_fpga/lab7radiant/impl_1/lab7radiant_impl_1.ldc]

} out]} {
   ::radiant::runengine::runtime_log $out
   exit 1
}
