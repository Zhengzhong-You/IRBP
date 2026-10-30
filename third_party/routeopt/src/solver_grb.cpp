/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#include "solver.hpp"

namespace RouteOpt {
#if SOLVER_TYPE == 0

    int Solver::getSlack(int first, int len, double *const values) const {
        auto error = GRBgetdblattrarray(model, "slack", first, len, values);
        return error;
    }

    int Solver::getDual(int first, int len, double *const values) const {
        auto error = GRBgetdblattrarray(model, "pi", first, len, values);
        return error;
    }

    int Solver::getCrossOver(int *const valueP) const {
        return GRBgetintparam(GRBgetenv(model), "Crossover", valueP);
    }

    int Solver::getNumRow(int *const valueP) const {
        auto error = GRBgetintattr(model, "NumConstrs", valueP);
        return error;
    }

    int Solver::getNumCol(int *const valueP) const {
        auto error = GRBgetintattr(model, "NumVars", valueP);
        return error;
    }

    int Solver::getStatus(int *const valueP) const {
        auto error = GRBgetintattr(model, "Status", valueP);
        return error;
    }

    int Solver::delConstraints(int len, int *const cind) {
        auto error = GRBdelconstrs(model, len, cind);
        return error;
    }

    int Solver::reoptimize(int method) const {
        int error;
        int now_method;
        error = getEnvMethod(&now_method);
        if (error) return error;
        if (now_method != method) {
            error = setEnvMethod(method);
            if (error) return error;
        }
        error = GRBoptimize(model);
        if (error) return error;
        if (now_method != method) {
            error = setEnvMethod(now_method);
            if (error) return error;
        }
        return error;
    }

    int Solver::optimize() {
        auto error = GRBoptimize(model);
        return error;
    }

    int Solver::mipOptimize() {
        auto error = GRBoptimize(model);
        if (solution_callback_error) std::rethrow_exception(solution_callback_error);
        return error;
    }

    SOLVERmodel *Solver::copyModel() const {
        auto newModel = GRBcopymodel(model);
        return newModel;
    }

    int Solver::freeModel() {
        if (!model) return 0;
        auto error = GRBfreemodel(model);
        model = nullptr;
        return error;
    }

    int Solver::loadEnv(const char *const logfilename) {
        auto error = GRBloadenv(&env, logfilename);
        return error;
    }

    int Solver::setEnvThreads(int value, bool if_model_free) {
        int error;
        if (if_model_free)
            error = GRBsetintparam(env, "Threads", value);
        else
            error = GRBsetintparam(GRBgetenv(model), "Threads", value);
        return error;
    }

    int Solver::getThreads(int *const valueP) const {
        auto error = GRBgetintparam(GRBgetenv(model), "Threads", valueP);
        return error;
    }

    int Solver::setEnvOutputFlag(int value, bool if_model_free) {
        int error;
        if (if_model_free)
            error = GRBsetintparam(env, "OutputFlag", value);
        else
            error = GRBsetintparam(GRBgetenv(model), "OutputFlag", value);
        return error;
    }

    int Solver::setEnvInfUnbdInfo(int value, bool if_model_free) {
        int error;
        if (if_model_free)
            error = GRBsetintparam(env, "InfUnbdInfo", value);
        else
            error = GRBsetintparam(GRBgetenv(model), "InfUnbdInfo", value);
        return error;
    }

    int Solver::setEnvMIPGap(double value, bool if_model_free) {
        int error;
        if (if_model_free)
            error = GRBsetdblparam(env, "MIPGap", value);
        else
            error = GRBsetdblparam(GRBgetenv(model), "MIPGap", value);
        return error;
    }

    int Solver::setEnvFeasibilityTol(double value, bool if_model_free) {
        int error;
        if (if_model_free)
            error = GRBsetdblparam(env, "FeasibilityTol", value);
        else
            error = GRBsetdblparam(GRBgetenv(model), "FeasibilityTol", value);
        return error;
    }

    int Solver::setEnvIntFeasTol(double value, bool if_model_free) {
        int error;
        if (if_model_free)
            error = GRBsetdblparam(env, "IntFeasTol", value);
        else
            error = GRBsetdblparam(GRBgetenv(model), "IntFeasTol", value);
        return error;
    }

    int Solver::newModel(const char *const Pname, int numvars,
                         double *const obj, double *const lb, double *const ub, char *const vtype,
                         char **const varnames) {
        auto error = GRBnewmodel(env, &model, Pname, numvars, obj, lb, ub, vtype, varnames);
        return error;
    }

    int Solver::addConstraints(int numconstrs, int numnz,
                               int *const cbeg, int *const cind, double *const cval,
                               char *const sense, double *const rhs, char **const constrnames) {
        auto error = GRBaddconstrs(model, numconstrs, numnz, cbeg, cind, cval, sense, rhs, constrnames);
        return error;
    }

    int Solver::updateModel() {
        auto error = GRBupdatemodel(model);
        return error;
    }

    int Solver::write(const char *filename) const {
        auto error = GRBwrite(model, filename);
        return error;
    }

    int Solver::read(const char *filename) const {
        return GRBread(model, filename);
    }

    int Solver::delVars(int len, int *const ind) {
        auto error = GRBdelvars(model, len, ind);
        return error;
    }

    void Solver::freeEnv() {
        if (env) GRBfreeenv(env);
        env = nullptr;
    }

    int Solver::getObjVal(double *const valueP) const {
        auto error = GRBgetdblattr(model, "ObjVal", valueP);
        return error;
    }

    int Solver::getObj(int first, int len, double *const values) const {
        auto error = GRBgetdblattrarray(model, "Obj", first, len, values);
        return error;
    }

    int Solver::getRhs(int first, int len, double *const values) const {
        auto error = GRBgetdblattrarray(model, "rhs", first, len, values);
        return error;
    }

    int Solver::getX(int first, int len, double *const values) const {
        auto error = GRBgetdblattrarray(model, "X", first, len, values);
        return error;
    }

    void Solver::getSolver(Solver *const solver) {
        model = solver->copyModel();
        env = GRBgetenv(model);
    }

    void Solver::getEnv(Solver *const solver) {
        env = solver->passEnv();
    }

    SOLVERenv *Solver::passEnv() const {
        auto error = env;
        return error;
    }

    int Solver::getRC(int first, int len, double *const values) const {
        auto error = GRBgetdblattrarray(model, "rc", first, len, values);
        return error;
    }

    int Solver::changeObj(int first, int len, double *const values) {
        auto error = GRBsetdblattrarray(model, "Obj", first, len, values);
        return error;
    }

    int Solver::setRhs(int first, int len, char *sense, double *const values) {
        auto error = GRBsetdblattrarray(model, "rhs", first, len, values);
        error += GRBsetcharattrarray(model, "Sense", first, len, sense);
        return error;
    }

    int Solver::setRhs(int first, int len, double *const values) {
        auto error = GRBsetdblattrarray(model, "rhs", first, len, values);
        return error;
    }

    int Solver::changeCoeffs(int cnt, int *const cind, int *const vind, double *const val) {
        auto error = GRBchgcoeffs(model, cnt, cind, vind, val);
        return error;
    }

    int Solver::XchangeCoeffs(size_t cnt, int *const cind, int *const vind, double *const val) {
        auto error = GRBXchgcoeffs(model, cnt, cind, vind, val);
        return error;
    }

    int Solver::addConstraint(int numnz, int *const cind, double *const cval, char sense, double rhs,
                              const char *const constrname) {
        auto error = GRBaddconstr(model, numnz, cind, cval, sense, rhs, constrname);
        return error;
    }

    int
    Solver::addVars(int numvars, int numnz, int *const vbeg, int *const vind, double *const vval, double *const obj,
                    double *const lb, double *const ub, char *const vtype, char **const varnames) {
        auto error = GRBaddvars(model, numvars, numnz, vbeg, vind, vval, obj, lb, ub, vtype, varnames);
        return error;
    }

    int
    Solver::addVar(int numnz, int *const vind, double *const vval, double obj, double lb, double ub, char vtype,
                   const char *const varname) {
        return GRBaddvar(model, numnz, vind, vval, obj, lb, ub, vtype, varname);
    }

    int
    Solver::XaddVars(int numvars,
                     size_t numnz,
                     size_t *const vbeg,
                     int *const vind,
                     double *const vval,
                     double *const obj,
                     double *const lb,
                     double *const ub,
                     char *const vtype,
                     char **const varnames) {
        auto error = GRBXaddvars(model, numvars, numnz, vbeg, vind, vval, obj, lb, ub, vtype, varnames);
        return error;
    }

    int Solver::XaddConstraints(int numconstrs, size_t numnz,
                                size_t *const cbeg, int *const cind, double *const cval,
                                char *const sense, double *const rhs, char **const constrnames) {
        auto error = GRBXaddconstrs(model, numconstrs, numnz, cbeg, cind, cval, sense, rhs, constrnames);
        return error;
    }

    int
    Solver::getConstraints(int *const numnzP,
                           int *const cbeg,
                           int *const cind,
                           double *const cval,
                           int start,
                           int len) const {
        auto error = GRBgetconstrs(model, numnzP, cbeg, cind, cval, start, len);
        return error;
    }

    int
    Solver::getVars(int *const numnzP,
                    int *const vbeg,
                    int *const vind,
                    double *const vval,
                    int start,
                    int len) const {
        return GRBgetvars(model, numnzP, vbeg, vind, vval, start, len);
    }

    int Solver::XgetConstraints(size_t *const numnzP, size_t *const cbeg,
                                int *const cind, double *const cval, int start, int len) const {
        auto error = GRBXgetconstrs(model, numnzP, cbeg, cind, cval, start, len);
        return error;
    }

    int Solver::XgetVars(size_t *const numnzP, size_t *const vbeg, int *const vind, double *const vval,
                         int start, int len) const {
        auto error = GRBXgetvars(model, numnzP, vbeg, vind, vval, start, len);
        return error;
    }

    int Solver::setEnvCrossOver(int value) const {
        return GRBsetintparam(GRBgetenv(model), "Crossover", value);
    }

    int Solver::setEnvCutoff(double value) {
        auto error = GRBsetdblparam(GRBgetenv(model), "Cutoff", value);
        return error;
    }

    int Solver::setVTypeArray(int first, int len, char *const newvalues) {
        auto error = GRBsetcharattrarray(model, "VType", first, len, newvalues);
        return error;
    }

    int Solver::setEnvMethod(int value) const {
        auto error = GRBsetintparam(GRBgetenv(model), "Method", value);
        return error;
    }

    int Solver::getEnvMethod(int *value) const {
        auto error = GRBgetintparam(GRBgetenv(model), "Method", value);
        return error;
    }

    int Solver::setEnvTimeLimit(double value) {
        auto error = GRBsetdblparam(GRBgetenv(model), "TimeLimit", value);
        return error;
    }

    int Solver::setModelSense(int value) {
        auto error = GRBsetintattr(model, "ModelSense", value);
        return error;
    }

    int Solver::setVBasis(int first, int len, int *vbasis) {
        auto error = GRBsetintattrarray(model, "VBasis", first, len, vbasis);
        return error;
    }

    int Solver::setCBasis(int first, int len, int *cbasis) {
        auto error = GRBsetintattrarray(model, "CBasis", first, len, cbasis);
        return error;
    }

    int Solver::getVBasis(int first, int len, int *vbasis) const {
        auto error = GRBgetintattrarray(model, "VBasis", first, len, vbasis);
        return error;
    }

    int Solver::getCBasis(int first, int len, int *cbasis) const {
        auto error = GRBgetintattrarray(model, "CBasis", first, len, cbasis);
        return error;
    }

    int Solver::getSense(int first, int len, char *sense) {
        auto error = GRBgetcharattrarray(model, "Sense", first, len, sense);
        return error;
    }

    int Solver::setColLower(int col, double value) {
        return GRBsetdblattrelement(model, GRB_DBL_ATTR_LB, col, value);
    }

    int Solver::setColUpper(int col, double value) {
        return GRBsetdblattrelement(model, GRB_DBL_ATTR_UB, col, value);
    }

    int Solver::getColLower(int col, double *value) const {
        return GRBgetdblattrelement(model, GRB_DBL_ATTR_LB, col, value);
    }

    int Solver::getColUpper(int col, double *value) const {
        return GRBgetdblattrelement(model, GRB_DBL_ATTR_UB, col, value);
    }

    int Solver::removeColLower(int col) {
        return GRBsetdblattrelement(model, GRB_DBL_ATTR_LB, col, -GRB_INFINITY);
    }

    int Solver::removeColUpper(int col) {
        return GRBsetdblattrelement(model, GRB_DBL_ATTR_UB, col, GRB_INFINITY);
    }

    int Solver::getCoeff(int row, int col, double &value) const {
        return GRBgetcoeff(model, row, col, &value);
    }

    int Solver::getSolCount(int *valueP) const {
        return GRBgetintattr(model, "SolCount", valueP);
    }

    int Solver::getObjFromPool(int sol, double *valueP) const {
        auto error = GRBsetintparam(GRBgetenv(model), "SolutionNumber", sol);
        error += GRBgetdblattr(model, "PoolObjVal", valueP);
        return error;
    }

    int Solver::getSolFromPool(int i, int first, int len, double *values) const {
        const int error = GRBsetintparam(GRBgetenv(model), "SolutionNumber", i);
        if (error) return error;
        return GRBgetdblattrarray(model, "Xn", first, len, values);
    }

    int Solver::readModel(const char *filename) {
        return GRBreadmodel(env, filename, &model);
    }
    int Solver::createEnv() {
        int error = GRBemptyenv(&env);
        if (error) return error;
        error = GRBsetintparam(env, "OutputFlag", 0);
        return error ? error : GRBstartenv(env);
    }
    int Solver::copyModelTo(Solver &target) const {
        return GRBcopymodeltoenv(model, target.env, &target.model);
    }
    int Solver::setObjConstant(double value) { return GRBsetdblattr(model, "ObjCon", value); }
    int Solver::setEnvSeed(int value) { return GRBsetintparam(GRBgetenv(model), "Seed", value); }
    int Solver::setEnvOptimalityTol(double value) { return GRBsetdblparam(GRBgetenv(model), "OptimalityTol", value); }
    int Solver::setEnvDualReductions(int value) { return GRBsetintparam(GRBgetenv(model), "DualReductions", value); }
    int Solver::setEnvQuad(int value) { return GRBsetintparam(GRBgetenv(model), "Quad", value); }
    int Solver::getFarkasDual(int first, int len, double *values) const { return GRBgetdblattrarray(model, "FarkasDual", first, len, values); }
    int Solver::setEnvBarConvTol(double value) { return GRBsetdblparam(GRBgetenv(model), "BarConvTol", value); }
    int Solver::setEnvPerturbValue(double value) { return GRBsetdblparam(GRBgetenv(model), "PerturbValue", value); }
    int Solver::setEnvPoolSolutions(int value) { return GRBsetintparam(GRBgetenv(model), "PoolSolutions", value); }
    int Solver::setEnvPoolSearchMode(int value) { return GRBsetintparam(GRBgetenv(model), "PoolSearchMode", value); }
    int Solver::setEnvMIPGapAbs(double value) { return GRBsetdblparam(GRBgetenv(model), "MIPGapAbs", value); }
    int Solver::setEnvSoftMemLimit(double value) { return GRBsetdblparam(GRBgetenv(model), "SoftMemLimit", value); }
    int Solver::setEnvLPWarmStart(int value) { return GRBsetintparam(GRBgetenv(model), "LPWarmStart", value); }
    int Solver::setEnvNodeLimit(double value) { return GRBsetdblparam(GRBgetenv(model), "NodeLimit", value); }
    int Solver::setEnvBestBdStop(double value) { return GRBsetdblparam(GRBgetenv(model), "BestBdStop", value); }
    int Solver::setEnvObjScale(double value) { return GRBsetdblparam(GRBgetenv(model), "ObjScale", value); }
    int Solver::setEnvMIPFocus(int value) { return GRBsetintparam(GRBgetenv(model), "MIPFocus", value); }
    int Solver::setEnvIntegralityFocus(int value) { return GRBsetintparam(GRBgetenv(model), "IntegralityFocus", value); }
    int Solver::getEnvIntegralityFocus(int *value) const { return GRBgetintparam(GRBgetenv(model), "IntegralityFocus", value); }
    int Solver::setEnvCuts(int value) { return GRBsetintparam(GRBgetenv(model), "Cuts", value); }
    int Solver::setMIPStart(int first, int len, const double *values) { return GRBsetdblattrarray(model, "Start", first, len, const_cast<double *>(values)); }
    int Solver::clearMIPStarts() { return GRBsetintattr(model, "NumStart", 0); }
    int Solver::getObjBound(double *value) const { return GRBgetdblattr(model, "ObjBound", value); }
    int Solver::getNodeCount(double *value) const { return GRBgetdblattr(model, "NodeCount", value); }
    int Solver::setIncumbentCallback(void (*callback)(double, void *), void *data) {
        incumbent_callback = callback;
        incumbent_data = data;
        return GRBsetcallbackfunc(model, [](GRBmodel *, void *context, int where, void *user) {
            auto &solver = *static_cast<Solver *>(user);
            if (where == GRB_CB_MIPSOL && solver.incumbent_callback) {
                double objective;
                const int error = GRBcbget(context, where, GRB_CB_MIPSOL_OBJ, &objective);
                if (error) return error;
                solver.incumbent_callback(objective, solver.incumbent_data);
            }
            return 0;
        }, this);
    }
    int Solver::setSolutionCallback(int columns, double incumbent,
                                    std::function<void(double, const std::vector<double> &)> callback,
                                    const double *validated_incumbent) {
        solution_columns = columns;
        solution_best = incumbent;
        solution_callback = std::move(callback);
        solution_callback_error = nullptr;
        solution_validated_incumbent = validated_incumbent;
        return GRBsetcallbackfunc(model, [](GRBmodel *model, void *context, int where, void *user) {
            auto &solver = *static_cast<Solver *>(user);
            if (where != GRB_CB_MIPSOL || !solver.solution_callback) return 0;
            double objective;
            int error = GRBcbget(context, where, GRB_CB_MIPSOL_OBJ, &objective);
            if (error || objective >= solver.solution_best) return error;
            try {
                std::vector<double> values(solver.solution_columns);
                error = GRBcbget(context, where, GRB_CB_MIPSOL_SOL, values.data());
                if (error) return error;
                solver.solution_callback(objective, values);
                solver.solution_best = solver.solution_validated_incumbent ? *solver.solution_validated_incumbent : objective;
            } catch (...) {
                // Never unwind a C solver callback. Stop and rethrow at the interface boundary.
                solver.solution_callback_error = std::current_exception();
                GRBterminate(model);
            }
            return 0;
        }, this);
    }
    const char *Solver::backendName() { return "gurobi"; }
    std::array<int, 3> Solver::version() {
        std::array<int, 3> result{};
        GRBversion(&result[0], &result[1], &result[2]);
        return result;
    }
#endif
}
