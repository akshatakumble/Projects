#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <climits>
#include <chrono>

std::vector<int> max_min_grouping(const std::vector<int>& A, int N, int M) {
    // define the DP tables
    std::vector<std::vector<int>> C(N + 1, std::vector<int>(M + 1, 0));
    std::vector<std::vector<int>> P(N + 1, std::vector<int>(M + 1, 0));

    // base case (one group)
    for (int i = 1; i <= N; ++i) {
        C[i][1] = std::accumulate(A.begin(), A.begin() + i, 0);
    }

    // fill up C and P tables using DP
    for (int j = 2; j <= M; ++j) {
        for (int i = j; i <= N; ++i) {
            C[i][j] = INT_MIN;
            int sum = 0;

            // partition from i-1 down to j-1
            for (int k = i - 1; k >= j - 1; --k) {
                sum += A[k];
                int min_val = std::min(C[k][j - 1], sum);

                if (C[i][j] < min_val) {
                    C[i][j] = min_val;
                    P[i][j] = k;
                }
            }
        }
    }

    // print Table C
    std::cout << "Table C (Max-Min Values):\n";
    for (int j = 1; j <= M; ++j) {
        for (int i = 1; i <= N; ++i) {
            std::cout << (C[i][j] == INT_MIN ? "-" : std::to_string(C[i][j])) << "\t";
        }
        std::cout << "\n";
    }

    // print Table P
    std::cout << "Table P (Partition Points):\n";
    for (int j = 1; j <= M; ++j) {
        for (int i = 1; i <= N; ++i) {
            std::cout << (P[i][j] == 0 ? "-" : std::to_string(P[i][j])) << "\t";
        }
        std::cout << "\n";
    }

    // backtrack to find the optimal grouping sizes
    std::vector<int> G(M);
    int current = N;
    for (int j = M; j >= 1; --j) {
        G[j - 1] = current - P[current][j];
        current = P[current][j];
    }

    // print group sizes (G_optimal)
    std::cout << "Optimal Group Sizes G: ";
    for (int size : G) {
        std::cout << size << " ";
    }
    std::cout << "\n";

    // calculate the sums of each group in B
    std::vector<int> B(M);
    int start = 0;
    for (int i = 0; i < M; ++i) {
        B[i] = std::accumulate(A.begin() + start, A.begin() + start + G[i], 0);
        start += G[i];
    }

    // print the sums of each group in B (B_min)
    std::cout << "Sum of each group B: ";
    for (int sum : B) {
        std::cout << sum << " ";
    }
    std::cout << "\n";

    // print the maximized minimum sum
    std::cout << "Maximized minimum sum: " << C[N][M] << "\n";

    return G;
}

int main() {
    int N, M;

    // input array size and number of groups
    std::cout << "Enter the number of elements in the array (N): ";
    std::cin >> N;

    std::cout << "Enter the number of groups (M): ";
    std::cin >> M;

    std::vector<int> A(N);
    std::cout << "Enter the elements of the array:\n";
    for (int i = 0; i < N; ++i) {
        std::cin >> A[i];
    }

    // measure the time taken by the function
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<int> optimal_grouping = max_min_grouping(A, N, M);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Execution time: " << elapsed.count() << " seconds\n";
    std::cout << "Press Enter to exit...";
    std::cin.ignore();
    std::cin.get();

    return 0;
}
