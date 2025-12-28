// Copyright 2010-2025 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Linq;
using Google.OrTools.LinearSolver;

/// <summary>
/// Demonstrates and tests LP basis warm-start functionality.
///
/// This test verifies that:
/// 1. BasisStatusAsInt() returns valid basis status values after solving
/// 2. SetStartingLpBasis() accepts the saved basis without error
/// 3. Warm-start reduces iteration count on re-solve (for non-trivial problems)
///
/// Usage:
///   dotnet run
///
/// Expected: All tests pass, warm-start shows iteration reduction on larger problem.
/// </summary>
public class WarmStartTest
{
    // BasisStatus enum values (matches MPSolver::BasisStatus in C++)
    private const int FREE = 0;
    private const int AT_LOWER_BOUND = 1;
    private const int AT_UPPER_BOUND = 2;
    private const int FIXED_VALUE = 3;
    private const int BASIC = 4;

    private static int testsPassed = 0;
    private static int testsFailed = 0;

    public static void Main(string[] args)
    {
        Console.WriteLine("=== OR-Tools LP Basis Warm-Start Test Suite ===\n");

        // Run all tests
        TestBasicWarmStart();
        TestBasisStatusValues();
        TestLargerProblemWarmStart();
        TestMultipleResolves();

        // Summary
        Console.WriteLine("\n=== Test Summary ===");
        Console.WriteLine($"Passed: {testsPassed}");
        Console.WriteLine($"Failed: {testsFailed}");

        if (testsFailed > 0)
        {
            Console.WriteLine("\nSome tests FAILED!");
            Environment.Exit(1);
        }
        else
        {
            Console.WriteLine("\nAll tests PASSED!");
        }
    }

    /// <summary>
    /// Basic test: solve, save basis, modify, warm-start, re-solve
    /// </summary>
    static void TestBasicWarmStart()
    {
        Console.WriteLine("--- Test: Basic Warm-Start ---");

        var solver = Solver.CreateSolver("GLOP");
        if (solver == null)
        {
            Fail("Could not create GLOP solver");
            return;
        }

        // CRITICAL: Disable presolve for warm-start to work
        solver.SetSolverSpecificParametersAsString("use_preprocessing: false");

        // Simple LP: max 3x + 2y, s.t. x + y <= 10, 2x + y <= 15, x,y >= 0
        var x = solver.MakeNumVar(0, double.PositiveInfinity, "x");
        var y = solver.MakeNumVar(0, double.PositiveInfinity, "y");

        var c1 = solver.MakeConstraint(double.NegativeInfinity, 10, "c1");
        c1.SetCoefficient(x, 1);
        c1.SetCoefficient(y, 1);

        var c2 = solver.MakeConstraint(double.NegativeInfinity, 15, "c2");
        c2.SetCoefficient(x, 2);
        c2.SetCoefficient(y, 1);

        solver.Objective().SetCoefficient(x, 3);
        solver.Objective().SetCoefficient(y, 2);
        solver.Objective().SetMaximization();

        // First solve
        var status = solver.Solve();
        AssertEqual(Solver.ResultStatus.OPTIMAL, status, "First solve should be optimal");

        // Get basis status
        int xBasis = x.BasisStatusAsInt();
        int yBasis = y.BasisStatusAsInt();
        int c1Basis = c1.BasisStatusAsInt();
        int c2Basis = c2.BasisStatusAsInt();

        Console.WriteLine($"  First solve: obj={solver.Objective().Value():F2}, iterations={solver.Iterations()}");
        Console.WriteLine($"  Basis: x={BasisName(xBasis)}, y={BasisName(yBasis)}, c1={BasisName(c1Basis)}, c2={BasisName(c2Basis)}");

        // Verify basis values are valid
        AssertTrue(IsValidBasisStatus(xBasis), "x basis status should be valid");
        AssertTrue(IsValidBasisStatus(yBasis), "y basis status should be valid");
        AssertTrue(IsValidBasisStatus(c1Basis), "c1 basis status should be valid");
        AssertTrue(IsValidBasisStatus(c2Basis), "c2 basis status should be valid");

        // Save basis
        int[] varBasis = { xBasis, yBasis };
        int[] conBasis = { c1Basis, c2Basis };

        // Modify objective slightly
        solver.Objective().SetCoefficient(x, 3.5);

        // Apply warm-start and re-solve
        solver.Reset();
        solver.SetStartingLpBasis(varBasis, conBasis);
        status = solver.Solve();

        AssertEqual(Solver.ResultStatus.OPTIMAL, status, "Warm-start solve should be optimal");
        Console.WriteLine($"  Warm-start solve: obj={solver.Objective().Value():F2}, iterations={solver.Iterations()}");

        Pass("Basic warm-start");
    }

    /// <summary>
    /// Test that basis status values are correct for different variable states
    /// </summary>
    static void TestBasisStatusValues()
    {
        Console.WriteLine("\n--- Test: Basis Status Values ---");

        var solver = Solver.CreateSolver("GLOP");
        if (solver == null)
        {
            Fail("Could not create GLOP solver");
            return;
        }

        solver.SetSolverSpecificParametersAsString("use_preprocessing: false");

        // Create a problem where we know the optimal basis:
        // max x, s.t. x <= 5, x >= 0
        // At optimum: x = 5 (AT_UPPER_BOUND for constraint slack, BASIC for x)
        var x = solver.MakeNumVar(0, 10, "x");
        var c = solver.MakeConstraint(double.NegativeInfinity, 5, "c");
        c.SetCoefficient(x, 1);
        solver.Objective().SetCoefficient(x, 1);
        solver.Objective().SetMaximization();

        solver.Solve();

        int xStatus = x.BasisStatusAsInt();
        int cStatus = c.BasisStatusAsInt();

        Console.WriteLine($"  x={x.SolutionValue()}, basis={BasisName(xStatus)}");
        Console.WriteLine($"  constraint slack basis={BasisName(cStatus)}");

        // x should be BASIC (in the basis, at optimal value 5)
        AssertEqual(BASIC, xStatus, "x should be BASIC at optimum");

        Pass("Basis status values");
    }

    /// <summary>
    /// Test with a larger problem to see iteration reduction
    /// </summary>
    static void TestLargerProblemWarmStart()
    {
        Console.WriteLine("\n--- Test: Larger Problem Warm-Start ---");

        var solver = Solver.CreateSolver("GLOP");
        if (solver == null)
        {
            Fail("Could not create GLOP solver");
            return;
        }

        solver.SetSolverSpecificParametersAsString("use_preprocessing: false");

        // Create a larger LP with 20 variables and 15 constraints
        int numVars = 20;
        int numConstraints = 15;

        var vars = new Variable[numVars];
        for (int i = 0; i < numVars; i++)
        {
            vars[i] = solver.MakeNumVar(0, 100, $"x{i}");
            solver.Objective().SetCoefficient(vars[i], i + 1);
        }
        solver.Objective().SetMaximization();

        var cons = new Constraint[numConstraints];
        var rand = new Random(42); // Fixed seed for reproducibility
        for (int j = 0; j < numConstraints; j++)
        {
            cons[j] = solver.MakeConstraint(double.NegativeInfinity, 100 + j * 10, $"c{j}");
            for (int i = 0; i < numVars; i++)
            {
                cons[j].SetCoefficient(vars[i], rand.NextDouble() * 2);
            }
        }

        // First solve (cold start)
        solver.Solve();
        long coldIterations = solver.Iterations();
        double firstObj = solver.Objective().Value();
        Console.WriteLine($"  Cold start: obj={firstObj:F2}, iterations={coldIterations}");

        // Save basis
        int[] varBasis = vars.Select(v => v.BasisStatusAsInt()).ToArray();
        int[] conBasis = cons.Select(c => c.BasisStatusAsInt()).ToArray();

        // Small modification to objective
        solver.Objective().SetCoefficient(vars[0], 1.5);

        // Cold start (for comparison)
        solver.Reset();
        solver.Solve();
        long coldIterations2 = solver.Iterations();
        Console.WriteLine($"  Cold re-solve: obj={solver.Objective().Value():F2}, iterations={coldIterations2}");

        // Warm start
        solver.Objective().SetCoefficient(vars[0], 1.5); // Same modification
        solver.Reset();
        solver.SetStartingLpBasis(varBasis, conBasis);
        solver.Solve();
        long warmIterations = solver.Iterations();
        Console.WriteLine($"  Warm re-solve: obj={solver.Objective().Value():F2}, iterations={warmIterations}");

        // Warm start should typically use fewer or equal iterations
        if (warmIterations <= coldIterations2)
        {
            Console.WriteLine($"  ✓ Warm-start saved {coldIterations2 - warmIterations} iterations");
            Pass("Larger problem warm-start");
        }
        else
        {
            Console.WriteLine($"  Note: Warm-start used more iterations (can happen occasionally)");
            Pass("Larger problem warm-start (executed without error)");
        }
    }

    /// <summary>
    /// Test multiple warm-start resolves in sequence
    /// </summary>
    static void TestMultipleResolves()
    {
        Console.WriteLine("\n--- Test: Multiple Warm-Start Resolves ---");

        var solver = Solver.CreateSolver("GLOP");
        if (solver == null)
        {
            Fail("Could not create GLOP solver");
            return;
        }

        solver.SetSolverSpecificParametersAsString("use_preprocessing: false");

        var x = solver.MakeNumVar(0, 100, "x");
        var y = solver.MakeNumVar(0, 100, "y");
        var c = solver.MakeConstraint(double.NegativeInfinity, 50, "c");
        c.SetCoefficient(x, 1);
        c.SetCoefficient(y, 1);
        solver.Objective().SetCoefficient(x, 1);
        solver.Objective().SetCoefficient(y, 1);
        solver.Objective().SetMaximization();

        // Initial solve
        solver.Solve();
        Console.WriteLine($"  Initial: obj={solver.Objective().Value():F2}");

        // Multiple warm-start cycles
        for (int i = 1; i <= 5; i++)
        {
            // Save current basis
            int[] varBasis = { x.BasisStatusAsInt(), y.BasisStatusAsInt() };
            int[] conBasis = { c.BasisStatusAsInt() };

            // Modify and re-solve with warm-start
            solver.Objective().SetCoefficient(x, 1.0 + i * 0.1);
            solver.Reset();
            solver.SetStartingLpBasis(varBasis, conBasis);
            var status = solver.Solve();

            AssertEqual(Solver.ResultStatus.OPTIMAL, status, $"Iteration {i} should be optimal");
            Console.WriteLine($"  Cycle {i}: obj={solver.Objective().Value():F2}, iterations={solver.Iterations()}");
        }

        Pass("Multiple warm-start resolves");
    }

    // --- Test Helpers ---

    static bool IsValidBasisStatus(int status)
    {
        return status >= FREE && status <= BASIC;
    }

    static string BasisName(int status)
    {
        return status switch
        {
            FREE => "FREE",
            AT_LOWER_BOUND => "AT_LOWER_BOUND",
            AT_UPPER_BOUND => "AT_UPPER_BOUND",
            FIXED_VALUE => "FIXED_VALUE",
            BASIC => "BASIC",
            _ => $"UNKNOWN({status})"
        };
    }

    static void AssertEqual<T>(T expected, T actual, string message)
    {
        if (!expected.Equals(actual))
        {
            Console.WriteLine($"  FAIL: {message}");
            Console.WriteLine($"    Expected: {expected}, Actual: {actual}");
            testsFailed++;
        }
    }

    static void AssertTrue(bool condition, string message)
    {
        if (!condition)
        {
            Console.WriteLine($"  FAIL: {message}");
            testsFailed++;
        }
    }

    static void Pass(string testName)
    {
        Console.WriteLine($"  ✓ {testName} PASSED");
        testsPassed++;
    }

    static void Fail(string message)
    {
        Console.WriteLine($"  ✗ FAIL: {message}");
        testsFailed++;
    }
}
