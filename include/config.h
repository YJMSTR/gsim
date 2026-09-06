#ifndef CONFIG_H
#define CONFIG_H

struct Config {
  bool EnableDumpGraph;
  bool DumpGraphDot;
  bool DumpGraphJson;
  bool DumpAssignTree;
  bool DumpConstStatus;
  bool DumpMtScheduleJson;
  bool DumpMtCoarseRegionReport;
  bool DisableReplicationOpt;
  bool MtReportOnly;
  bool MtStableOutput;
  bool MtContextCache;
  bool MtReportTimers;
  std::string MtHelperMode;
  std::string MtBatchFormationMode;
  std::string MtCoarseRuntimeMode;
  std::string MtCoarseProfitabilityMode;
  std::string MtCoarseWorkerPolicyMode;
  std::string OutputDir;
  std::string InputBaseName;
  int SuperNodeMaxSize;
  uint32_t cppMaxSizeKB;
  int MtActiveFrequencyCostThreshold;
  std::string sep_module;
  std::string sep_aggr;
  int MergeWhenSize;
  int When2muxBound;
  int LogLevel;
  int NumThreads;
  std::set<std::string> DumpStages;
  Config();
};

extern Config globalConfig;

#endif
