#include <cstdio>
#include <cstdlib>
#include <vector>

struct TestData {
    int id;
    float val;
    TestData(int i, float v) : id(i), val(v) {}
};

void function2(TestData *d)
{
    d->val = 2.3;
}

void function1(std::vector<TestData> &data)
{
    function2(&data[2]);
}

int main(int argc, char **argv)
{
    std::vector<TestData> data;
    for (int a = 0; a < 5; ++a)
        data.push_back(TestData(a, 1.5));
    function1(data);
    for (int a = 0; a < (int) data.size(); ++a)
        printf("data id: %d, val: %f\n", data[a].id, data[a].val);
    
    /*std::vector<TestData> avec;
    
    for (int a = 0; a < 5; ++a)
        avec.push_back(TestData(a, 1.5));
    
    std::vector<TestData> bvec = avec;
    bvec.erase(bvec.begin() + 3);
    bvec.erase(bvec.begin() + 1);
    bvec[0].val = 2.3;
    bvec[2].val = 2.3;
    
    printf("avec.size: %ld, bvec.size: %ld\n", avec.size(), bvec.size());
    
    for (int a = 0; a < (int) avec.size(); ++a)
        printf("avec id: %d, val: %f\n", avec[a].id, avec[a].val);
    printf("\n");
    for (int b = 0; b < (int) bvec.size(); ++b)
        printf("bvec id: %d, val: %f\n", bvec[b].id, bvec[b].val);*/
    
    return 0;
}
