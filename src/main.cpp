/**
 * @file main.cpp
 * @brief start entry
 */

#include <stdio.h>

#include "common.h"
#include "graph.h"

#include <sstream>
#include <string>
#include <cerrno>
#include <cstdint>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef GSIM_VERSION
#define GSIM_VERSION "UNKNOWN"
#endif

#ifndef GSIM_BUILD_DATE
#define GSIM_BUILD_DATE "UNKNOWN"
#endif

#ifndef GSIM_CXX_VERSION
#define GSIM_CXX_VERSION "UNKNOWN"
#endif

PNode* parseFIR(char *strbuf);
void preorder_traversal(PNode* root);
graph* AST2Graph(PNode* root);
void inferAllWidth();

Config::Config() {
  EnableDumpGraph = false;
  DumpGraphDot = false;
  DumpGraphJson = false;
  DumpAssignTree = false;
  DumpConstStatus = false;
  DumpMtScheduleJson = false;
  DumpMtRepCutLiteReport = false;
  DumpMtCoarseRegionReport = false;
  DisableReplicationOpt = false;
  MtReportOnly = false;
  MtStableOutput = false;
  MtContextCache = false;
  MtReportTimers = false;
  MtHelperMode = "off";
  MtRepCutLiteMode = "off";
  MtBatchFormationMode = "legacy";
  MtCoarseRuntimeMode = "layered";
  MtCoarseProfitabilityMode = "off";
  MtCoarseWorkerPolicyMode = "static";
  OutputDir = ".";
  InputBaseName = "";
  SuperNodeMaxSize = 35;
  cppMaxSizeKB = -1;
  MtRepCutCopyBudget = 0;
  MtRepCutFanoutBudget = 0;
  MtActiveFrequencyCostThreshold = 2;
  sep_module = "$";
  sep_aggr = "$$";
  MergeWhenSize = 5;
  When2muxBound = 2;
  LogLevel = 0;
}
Config globalConfig;

static std::set<std::string> parseStageList(const std::string& arg) {
  std::set<std::string> stages;
  std::stringstream ss(arg);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (!token.empty()) stages.insert(token);
  }
  return stages;
}

static inline bool shouldDumpStage(const std::string& name) {
  return globalConfig.DumpStages.empty() || globalConfig.DumpStages.count(name);
}

static std::string basenameNoExt(const char* path) {
  std::string name(path);
  size_t slash = name.find_last_of("/");
  if (slash != std::string::npos) name = name.substr(slash + 1);
  size_t dot = name.find_last_of('.');
  if (dot != std::string::npos) name = name.substr(0, dot);
  return name;
}

static bool parseNonNegativeInt(const char* text, int& value) {
  if (text == nullptr || text[0] == '\0') return false;
  char* end = nullptr;
  errno = 0;
  long parsed = strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed < 0 || parsed > INT32_MAX) return false;
  value = static_cast<int>(parsed);
  return true;
}


#define FUNC_WRAPPER_INTERNAL(func, name, dumpCond) \
  do { \
    if (globalConfig.LogLevel > 0 && strlen(name) != 0) { \
      fprintf(stderr, "[GSIM] %s begin\n", name); \
      fflush(stderr); \
    } \
    struct timeval start = getTime(); \
    func; \
    struct timeval end = getTime(); \
    const char* label = strlen(name) ? name : "{" #func "}"; \
    showTime(label, start, end); \
    if (globalConfig.LogLevel > 0 && strlen(name) != 0) { \
      fprintf(stderr, "[GSIM] %s done\n", name); \
      fflush(stderr); \
    } \
    if (dumpCond && globalConfig.EnableDumpGraph && shouldDumpStage(name)) g->dump(std::to_string(dumpIdx ++) + name); \
  } while(0)

#define FUNC_TIMER(func)         FUNC_WRAPPER_INTERNAL(func, "", false)
#define FUNC_WRAPPER(func, name) FUNC_WRAPPER_INTERNAL(func, name, true)

static void printVersionBrief() {
  std::cout << "GSIM " << GSIM_VERSION << "\n";
}

static void printVersionDetail() {
  std::cout << "GSIM " << GSIM_VERSION << "\n"
            << "Build Date: " << GSIM_BUILD_DATE << "\n"
            << "Build CXX: " << GSIM_CXX_VERSION << "\n";
}

static void printUsage(const char* ProgName) {
  printVersionBrief();
  std::cout << "Usage: " << ProgName << " [options] <input file>\n"
            << "Options:\n"
            << "  -h, --help                       Display this help message and exit.\n"
            << "  -v, --version                    Display version/build information and exit.\n"
            << "      --dump                       Enable dumping of the graph.\n"
            << "      --dir=[dir]                  Specify the output directory.\n"
            << "      --supernode-max-size=[num]   Specify the maximum size of a superNode.\n"
            << "      --cpp-max-size-KB=[num]      Specify the maximum size (approximate) of a generated C++ file.\n"
            << "      --sep-mod=[str]              Specify the seperator for submodule (default: $).\n"
            << "      --sep-aggr=[str]             Specify the seperator for aggregate member (default: $$).\n"
            << "      --when-size=[num]            Bound for merging nested when blocks (default: 5).\n"
            << "      --when2mux-bound=[num]       Bound for converting when to mux (default: 2).\n"
            << "      --log-level=[0|1|2]          Verbosity for additional logs.\n"
            << "      --dump-json                  Dump graphs in JSON (disable dot unless --dump-dot is also set).\n"
            << "      --dump-dot                   Dump graphs in DOT (disable json unless --dump-json is also set).\n"
            << "      --dump-stages=a,b,c          Dump only the listed stages (e.g., Init,TopoSort,AliasAnalysis).\n"
            << "      --dump-assign-tree           Include assignTree structure in JSON dump (can be large).\n"
            << "      --dump-const-status          Dump per-node constant-analysis status before removing constants.\n"
            << "      --dump-mt-schedule-json      Dump gsim-mt schedule metadata JSON without changing generated model behavior.\n"
            << "      --mt-helper-mode=off|seq|buffered-seq|mt|mt-level-dispatch\n"
            << "                                      Emit gsim-mt per-super helpers; off is the default.\n"
            << "                                      mt emits a bounded scan-order-preserving threaded helper runtime.\n"
            << "                                      mt-level-dispatch (28c Phase 1A) admits safe-serial cppIds into the\n"
            << "                                      coarse region path and routes worker0-only cppIds to the main thread.\n"
            << "                                      mt-level-dispatch implies coarse batch formation + static profitability\n"
            << "                                      unless the user already chose otherwise.\n"
            << "      --mt-repcut-lite=off|on       Enable bounded RepCut-lite candidate selection; off is the default.\n"
            << "      --mt-repcut-copy-budget=N     Total RepCut-lite copy cost budget; default 0.\n"
            << "      --mt-repcut-fanout-budget=N   Per-candidate RepCut-lite fanout budget; default 0.\n"
            << "      --mt-batch-formation=legacy|active-frequency|coarse\n"
            << "                                      Select pure batch formation mode; legacy is default.\n"
            << "      --mt-coarse-runtime=layered|mtask\n"
            << "                                      Select coarse runtime lowering; layered preserves the 18X runtime.\n"
            << "      --mt-coarse-profitability=off|static\n"
            << "                                      Select conservative coarse admission/worker policy; off preserves 19X.\n"
            << "      --mt-coarse-worker-policy=static|profitable\n"
            << "                                      Select coarse MTask worker assignment policy; static preserves 20X.\n"
            << "      --mt-active-frequency-cost-threshold=N\n"
            << "                                      Minimum estimated released static cost for active-frequency batches; default 2.\n"
            << "      --dump-mt-repcut-lite-report  Write a deterministic RepCut-lite candidate report JSON.\n"
            << "      --dump-mt-coarse-region-report\n"
            << "                                      Write a deterministic coarse-region report JSON.\n"
            << "      --disable-replication-opt     Skip the existing gsim replicationOpt pass.\n"
            << "      --mt-report-only              Stop after MT report JSON generation; skip generated C++ emission.\n"
            << "      --mt-stable-output            Preserve generated header/source mtimes when contents are unchanged.\n"
            << "      --mt-context-cache            Memoize MT task classification maps within one generation.\n"
            << "      --mt-report-timers           Print per-MT-report dump timings.\n"
            ;
}

static char* parseCommandLine(int argc, char** argv) {
  if (argc <= 1) {
    printUsage(argv[0]);
    std::cout.flush();
    fflush(nullptr);
    _exit(EXIT_SUCCESS); // avoid running atexit/destructors which crash in full-static
  }

  enum LongOptionIndex {
    OPT_HELP = 0,
    OPT_VERSION,
    OPT_DUMP,
    OPT_DIR,
    OPT_SUPER_MAX,
    OPT_CPP_MAX,
    OPT_SEP_MOD,
    OPT_SEP_AGGR,
    OPT_WHEN_SIZE,
    OPT_WHEN2MUX,
    OPT_LOG_LEVEL,
    OPT_DUMP_JSON,
    OPT_DUMP_DOT,
    OPT_DUMP_STAGES,
    OPT_DUMP_ASSIGN_TREE,
    OPT_DUMP_CONST_STATUS,
    OPT_DUMP_MT_SCHEDULE_JSON,
    OPT_MT_HELPER_MODE,
    OPT_MT_REPCUT_LITE,
    OPT_MT_REPCUT_COPY_BUDGET,
    OPT_MT_REPCUT_FANOUT_BUDGET,
    OPT_MT_BATCH_FORMATION,
    OPT_MT_COARSE_RUNTIME,
    OPT_MT_COARSE_PROFITABILITY,
    OPT_MT_COARSE_WORKER_POLICY,
    OPT_MT_ACTIVE_FREQUENCY_COST_THRESHOLD,
    OPT_DUMP_MT_REPCUT_LITE_REPORT,
    OPT_DUMP_MT_COARSE_REGION_REPORT,
    OPT_DISABLE_REPLICATION_OPT,
    OPT_MT_REPORT_ONLY,
    OPT_MT_STABLE_OUTPUT,
    OPT_MT_CONTEXT_CACHE,
    OPT_MT_REPORT_TIMERS,
  };

  const struct option Table[] = {
      {"help", no_argument, nullptr, 'h'},
      {"version", no_argument, nullptr, 'v'},
      {"dump", no_argument, nullptr, 'd'},
      {"dir", required_argument, nullptr, 0},
      {"supernode-max-size", required_argument, nullptr, 0},
      {"cpp-max-size-KB", required_argument, nullptr, 0},
      {"sep-mod", required_argument, nullptr, 0},
      {"sep-aggr", required_argument, nullptr, 0},
      {"when-size", required_argument, nullptr, 0},
      {"when2mux-bound", required_argument, nullptr, 0},
      {"log-level", required_argument, nullptr, 0},
      {"dump-json", no_argument, nullptr, 0},
      {"dump-dot", no_argument, nullptr, 0},
      {"dump-stages", required_argument, nullptr, 0},
      {"dump-assign-tree", no_argument, nullptr, 0},
      {"dump-const-status", no_argument, nullptr, 0},
      {"dump-mt-schedule-json", no_argument, nullptr, 0},
      {"mt-helper-mode", required_argument, nullptr, 0},
      {"mt-repcut-lite", required_argument, nullptr, 0},
      {"mt-repcut-copy-budget", required_argument, nullptr, 0},
      {"mt-repcut-fanout-budget", required_argument, nullptr, 0},
      {"mt-batch-formation", required_argument, nullptr, 0},
      {"mt-coarse-runtime", required_argument, nullptr, 0},
      {"mt-coarse-profitability", required_argument, nullptr, 0},
      {"mt-coarse-worker-policy", required_argument, nullptr, 0},
      {"mt-active-frequency-cost-threshold", required_argument, nullptr, 0},
      {"dump-mt-repcut-lite-report", no_argument, nullptr, 0},
      {"dump-mt-coarse-region-report", no_argument, nullptr, 0},
      {"disable-replication-opt", no_argument, nullptr, 0},
      {"mt-report-only", no_argument, nullptr, 0},
      {"mt-stable-output", no_argument, nullptr, 0},
      {"mt-context-cache", no_argument, nullptr, 0},
      {"mt-report-timers", no_argument, nullptr, 0},
      {nullptr, no_argument, nullptr, 0},
  };

  bool explicitJson = false;
  bool explicitDot = false;
  int Option{0};
  int option_index;
  while ((Option = getopt_long(argc, argv, "-hv", Table, &option_index)) != -1) {
    switch (Option) {
      case 0: switch (option_index) {
                case OPT_DUMP:
                  globalConfig.EnableDumpGraph = true;
                  globalConfig.DumpGraphDot = true;
                  globalConfig.DumpGraphJson = true;
                  explicitJson = explicitDot = false;
                  break;
                case OPT_DIR: globalConfig.OutputDir = optarg; break;
                case OPT_SUPER_MAX: sscanf(optarg, "%d", &globalConfig.SuperNodeMaxSize); break;
                case OPT_CPP_MAX: sscanf(optarg, "%d", &globalConfig.cppMaxSizeKB); break;
                case OPT_SEP_MOD: globalConfig.sep_module = optarg; break;
                case OPT_SEP_AGGR: globalConfig.sep_aggr = optarg; break;
                case OPT_WHEN_SIZE: sscanf(optarg, "%d", &globalConfig.MergeWhenSize); break;
                case OPT_WHEN2MUX: sscanf(optarg, "%d", &globalConfig.When2muxBound); break;
                case OPT_LOG_LEVEL: sscanf(optarg, "%d", &globalConfig.LogLevel); break;
                case OPT_DUMP_JSON:
                  if (explicitDot) {
                    fprintf(stderr, "Error: --dump-json and --dump-dot cannot be used together.\n");
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  globalConfig.EnableDumpGraph = true;
                  globalConfig.DumpGraphJson = true;
                  explicitJson = true;
                  globalConfig.DumpGraphDot = false;
                  break;
                case OPT_DUMP_DOT:
                  if (explicitJson) {
                    fprintf(stderr, "Error: --dump-json and --dump-dot cannot be used together.\n");
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  globalConfig.EnableDumpGraph = true;
                  globalConfig.DumpGraphDot = true;
                  explicitDot = true;
                  globalConfig.DumpGraphJson = false;
                  break;
                case OPT_DUMP_STAGES:
                  globalConfig.EnableDumpGraph = true;
                  globalConfig.DumpStages = parseStageList(optarg);
                  break;
                case OPT_DUMP_ASSIGN_TREE:
                  globalConfig.DumpAssignTree = true;
                  break;
                case OPT_DUMP_CONST_STATUS:
                  globalConfig.DumpConstStatus = true;
                  break;
                case OPT_DUMP_MT_SCHEDULE_JSON:
                  globalConfig.DumpMtScheduleJson = true;
                  break;
                case OPT_MT_HELPER_MODE:
                  globalConfig.MtHelperMode = optarg;
                  if (globalConfig.MtHelperMode != "off" &&
                      globalConfig.MtHelperMode != "seq" &&
                      globalConfig.MtHelperMode != "buffered-seq" &&
                      globalConfig.MtHelperMode != "mt" &&
                      globalConfig.MtHelperMode != "mt-level-dispatch") {
                    fprintf(stderr, "Error: unknown --mt-helper-mode '%s' (expected off, seq, buffered-seq, mt, or mt-level-dispatch).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_REPCUT_LITE:
                  globalConfig.MtRepCutLiteMode = optarg;
                  if (globalConfig.MtRepCutLiteMode != "off" &&
                      globalConfig.MtRepCutLiteMode != "on") {
                    fprintf(stderr, "Error: unknown --mt-repcut-lite '%s' (expected off or on).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_REPCUT_COPY_BUDGET:
                  if (!parseNonNegativeInt(optarg, globalConfig.MtRepCutCopyBudget)) {
                    fprintf(stderr, "Error: invalid --mt-repcut-copy-budget '%s' (expected non-negative integer).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_REPCUT_FANOUT_BUDGET:
                  if (!parseNonNegativeInt(optarg, globalConfig.MtRepCutFanoutBudget)) {
                    fprintf(stderr, "Error: invalid --mt-repcut-fanout-budget '%s' (expected non-negative integer).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_BATCH_FORMATION:
                  globalConfig.MtBatchFormationMode = optarg;
                  if (globalConfig.MtBatchFormationMode != "legacy" &&
                      globalConfig.MtBatchFormationMode != "active-frequency" &&
                      globalConfig.MtBatchFormationMode != "coarse") {
                    fprintf(stderr, "Error: unknown --mt-batch-formation '%s' (expected legacy, active-frequency, or coarse).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_COARSE_RUNTIME:
                  globalConfig.MtCoarseRuntimeMode = optarg;
                  if (globalConfig.MtCoarseRuntimeMode != "layered" &&
                      globalConfig.MtCoarseRuntimeMode != "mtask") {
                    fprintf(stderr, "Error: unknown --mt-coarse-runtime '%s' (expected layered or mtask).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_COARSE_PROFITABILITY:
                  globalConfig.MtCoarseProfitabilityMode = optarg;
                  if (globalConfig.MtCoarseProfitabilityMode != "off" &&
                      globalConfig.MtCoarseProfitabilityMode != "static") {
                    fprintf(stderr, "Error: unknown --mt-coarse-profitability '%s' (expected off or static).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_COARSE_WORKER_POLICY:
                  globalConfig.MtCoarseWorkerPolicyMode = optarg;
                  if (globalConfig.MtCoarseWorkerPolicyMode != "static" &&
                      globalConfig.MtCoarseWorkerPolicyMode != "profitable") {
                    fprintf(stderr, "Error: unknown --mt-coarse-worker-policy '%s' (expected static or profitable).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_MT_ACTIVE_FREQUENCY_COST_THRESHOLD:
                  if (!parseNonNegativeInt(optarg, globalConfig.MtActiveFrequencyCostThreshold)) {
                    fprintf(stderr, "Error: invalid --mt-active-frequency-cost-threshold '%s' (expected non-negative integer).\n", optarg);
                    printUsage(argv[0]);
                    std::cout.flush();
                    fflush(nullptr);
                    _exit(EXIT_FAILURE);
                  }
                  break;
                case OPT_DUMP_MT_REPCUT_LITE_REPORT:
                  globalConfig.DumpMtRepCutLiteReport = true;
                  break;
                case OPT_DUMP_MT_COARSE_REGION_REPORT:
                  globalConfig.DumpMtCoarseRegionReport = true;
                  break;
                case OPT_DISABLE_REPLICATION_OPT:
                  globalConfig.DisableReplicationOpt = true;
                  break;
                case OPT_MT_REPORT_ONLY:
                  globalConfig.MtReportOnly = true;
                  break;
                case OPT_MT_STABLE_OUTPUT:
                  globalConfig.MtStableOutput = true;
                  break;
                case OPT_MT_CONTEXT_CACHE:
                  globalConfig.MtContextCache = true;
                  break;
                case OPT_MT_REPORT_TIMERS:
                  globalConfig.MtReportTimers = true;
                  break;
                default: printUsage(argv[0]); std::cout.flush(); fflush(nullptr); _exit(EXIT_SUCCESS);
              }
              break;
      case 1:
        globalConfig.InputBaseName = basenameNoExt(optarg);
        return optarg; // InputFileName
      case 'd':
        globalConfig.EnableDumpGraph = true;
        globalConfig.DumpGraphDot = true;
        globalConfig.DumpGraphJson = true;
        break;
      case 'v':
        printVersionDetail();
        std::cout.flush();
        fflush(nullptr);
        _exit(EXIT_SUCCESS);
      case 'h':
      default: {
        printUsage(argv[0]);
        std::cout.flush();
        fflush(nullptr);
        _exit(EXIT_SUCCESS);
      }
    }
  }
  assert(0);
}

static char* readFile(const char *InputFileName, size_t &size, size_t &mapSize) {
  int fd = open(InputFileName, O_RDONLY);
  assert(fd != -1);
  struct stat sb;
  int ret = fstat(fd, &sb);
  assert(ret != -1);
  size = sb.st_size + 1;
  mapSize = (size + 4095) & ~4095L;
  char *buf = (char *)mmap(NULL, mapSize, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);
  assert(buf != (void *)-1);
  buf[size - 1] = '\0';
  close(fd);
  return buf;
}

/**
 * @brief main function.
 *
 * @param argc arg count.
 * @param argv arg value string.
 * @return exit state.
 */
int main(int argc, char** argv) {
  TIMER_START(total);
  graph* g = NULL;
  static int dumpIdx = 0;
  const char *InputFileName = parseCommandLine(argc, argv);

  // Standalone seed2 format transcode (GSIM_SEED2_TRANSCODE=in[:out]): convert a seed
  // to the v2 codec frame without any parsing/generation, then exit.
  if (std::getenv("GSIM_SEED2_TRANSCODE") != nullptr) return mtSeed2TranscodeMain();
  // Standalone canon v1/v2 divergence-detection fuzz (GSIM_SEED2_CANON_FUZZ=<iters>).
  if (std::getenv("GSIM_SEED2_CANON_FUZZ") != nullptr) return mtSeed2CanonFuzzMain();



  // 28c Phase 1A: mt-level-dispatch implies coarse formation + static profitability.
  // Promote only when user has not explicitly chosen otherwise (defaults from
  // Config::Config(): MtBatchFormationMode="legacy", MtCoarseProfitabilityMode="off").
  if (globalConfig.MtHelperMode == "mt-level-dispatch") {
    if (globalConfig.MtBatchFormationMode == "legacy") {
      globalConfig.MtBatchFormationMode = "coarse";
    }
    if (globalConfig.MtCoarseProfitabilityMode == "off") {
      globalConfig.MtCoarseProfitabilityMode = "static";
    }
    // Track 2 Week 5: cost-based LPT worker assignment measurably improves
    // dynamic runtime load balance, so default to profitable for mt-level-dispatch.
    if (globalConfig.MtCoarseWorkerPolicyMode == "static") {
      globalConfig.MtCoarseWorkerPolicyMode = "profitable";
    }
    // MtCoarseRuntimeMode default "layered" + MtCoarseWorkerPolicyMode default "static"
    // are exactly what 51-design §5.3 specifies for level-by-level dispatch.
  }

  size_t size = 0, mapSize = 0;
  char *strbuf;
  FUNC_TIMER(strbuf = readFile(InputFileName, size, mapSize));

  PNode* globalRoot;
  FUNC_TIMER(globalRoot= parseFIR(strbuf));
  munmap(strbuf, mapSize);

  FUNC_WRAPPER(g = AST2Graph(globalRoot), "Init");
  if (std::getenv("GSIM_DEBUG_PRE_CANON")) fprintf(stderr, "[pre-canon] ast2graph %016zx\n", (size_t)g->canonRawHash());

  FUNC_TIMER(g->splitArray());
  if (std::getenv("GSIM_DEBUG_PRE_CANON")) fprintf(stderr, "[pre-canon] splitArray %016zx\n", (size_t)g->canonRawHash());

  FUNC_TIMER(g->detectLoop());
  if (std::getenv("GSIM_DEBUG_PRE_CANON")) fprintf(stderr, "[pre-canon] detectLoop %016zx\n", (size_t)g->canonRawHash());

  FUNC_WRAPPER(g->topoSort(), "TopoSort");
  g->canonDumpTag("pass.topoSort");

  FUNC_TIMER(g->inferAllWidth());

  FUNC_WRAPPER(g->removeDeadNodes(), "RemoveDeadNodes");
  g->canonDumpTag("pass.removeDeadNodes");

  FUNC_WRAPPER(g->exprOpt(), "ExprOpt");
  g->canonDumpTag("pass.exprOpt");

  FUNC_TIMER(g->usedBits());

  FUNC_TIMER(g->splitNodes());

  if (globalConfig.EnableDumpGraph && shouldDumpStage("AfterSplitNodes")) g->dump(std::to_string(dumpIdx ++) + "AfterSplitNodes");

  FUNC_WRAPPER(g->removeDeadNodes(), "RemoveDeadNodes1");

  FUNC_WRAPPER(g->constantAnalysis(), "ConstantAnalysis");

  FUNC_WRAPPER(g->removeDeadNodes(), "RemoveDeadNodes");

  FUNC_WRAPPER(g->aliasAnalysis(), "AliasAnalysis");

  FUNC_WRAPPER(g->patternDetect(), "PatternDetect");

  FUNC_WRAPPER(g->commonExpr(), "CommonExpr");
  g->canonDumpTag("pass.commonExpr");

  FUNC_WRAPPER(g->removeDeadNodes(), "RemoveDeadNodes");
  g->canonDumpTag("pass.removeDeadNodes2");

  FUNC_WRAPPER(g->graphPartition(), "graphPartition");
  g->canonDumpTag("pass.graphPartition");

  if (!globalConfig.DisableReplicationOpt) {
    FUNC_WRAPPER(g->replicationOpt(), "Replication");
    // pin the replication outcome (content + order) like the graph passes.
    if (mtSeed2WriteActive()) g->canonSeed2Record("pass.replication");
    if (mtSeed2ReplayActive()) g->canonSeed2Apply("pass.replication");
    g->canonDumpTag("pass.replication");
  }

  // FUNC_WRAPPER(g->mergeRegister(), "MergeRegister");

  // FUNC_WRAPPER(g->constructRegs(), "ConstructRegs");
  FUNC_TIMER(g->generateStmtTree());

  FUNC_TIMER(g->instsGenerator());

  // final pin before emission - the generated source is a pure function of the
  // graph at this point; a canon match here means byte-identical output.
  if (mtSeed2WriteActive()) g->canonSeed2Record("pass.preEmit");
  if (mtSeed2ReplayActive()) g->canonSeed2Apply("pass.preEmit");
  g->canonDumpTag("pass.preEmit");



  FUNC_WRAPPER(g->cppEmitter(), "Final");

  TIMER_END(total);

  return 0;
}
